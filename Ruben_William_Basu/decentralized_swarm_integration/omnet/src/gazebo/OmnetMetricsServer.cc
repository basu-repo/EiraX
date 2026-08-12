// SPDX-License-Identifier: LGPL-3.0-or-later
#include "OmnetMetricsServer.h"

#define WANT_WINSOCK2
#include <omnetpp/platdep/sockets.h>
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32) || defined(__CYGWIN__) || defined(_WIN64)
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#endif

#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

#include "inet/common/INETMath.h"
#include "inet/common/packet/Packet.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/SignalTag_m.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using omnetpp::cMessage;
using omnetpp::cRuntimeError;
using omnetpp::cSimpleModule;

// ── Signal registration ────────────────────────────────────────────────────
const omnetpp::simsignal_t OmnetMetricsServer::packetSentToUpperSignal =
    omnetpp::cComponent::registerSignal("packetSentToUpper");
const omnetpp::simsignal_t OmnetMetricsServer::packetDroppedSignal =
    omnetpp::cComponent::registerSignal("packetDropped");
const omnetpp::simsignal_t OmnetMetricsServer::droppedPacketSignal =
    omnetpp::cComponent::registerSignal("droppedPacket");
const omnetpp::simsignal_t OmnetMetricsServer::loraAppPacketSentSignal =
    omnetpp::cComponent::registerSignal("LoRa_AppPacketSent");

Define_Module(OmnetMetricsServer);

// ── Helper: cast void* back to SOCKET ─────────────────────────────────────
static SOCKET sockFromPtr(void* ptr)
{
    return *static_cast<SOCKET*>(ptr);
}

// ── Lifecycle ─────────────────────────────────────────────────────────────

void OmnetMetricsServer::initialize(int stage)
{
    if (stage == inet::INITSTAGE_LOCAL) {
        port           = par("port");
        updateInterval = par("updateInterval");
        ugvModulePath  = par("ugvModule").stdstringValue();
        uavModulePath  = par("uavModule").stdstringValue();
        radioModulePath = par("radioModulePath").stdstringValue();
        txPowerDbm     = par("txPowerDbm");
        carrierFreqHz  = par("carrierFreqHz");
        noisePowerDbm  = par("noisePowerDbm");
        usePathLossFallback = par("usePathLossFallback");
        publishRadioDistanceEstimate = par("publishRadioDistanceEstimate");
        radioDistanceModel = par("radioDistanceModel").stdstringValue();
        loraPathLossD0M = par("loraPathLossD0");
        loraPathLossGamma = par("loraPathLossGamma");
        loraPathLossPLd0Db = par("loraPathLossPLd0Db");

        metricsTimer = new cMessage("omnetMetricsTick");
        scheduleAt(omnetpp::simTime() + updateInterval, metricsTimer);

        startServer();
    }
    else if (stage == inet::INITSTAGE_LAST) {
        // Resolve mobility modules for position queries.
        auto* ugvMod = getModuleByPath(ugvModulePath.c_str());
        auto* uavMod = getModuleByPath(uavModulePath.c_str());

        if (ugvMod)
            ugvMobility = omnetpp::check_and_cast<inet::IMobility*>(
                ugvMod->getSubmodule("mobility"));
        if (uavMod)
            uavMobility = omnetpp::check_and_cast<inet::IMobility*>(
                uavMod->getSubmodule("mobility"));

        if (!ugvMobility || !uavMobility)
            EV_WARN << "OmnetMetricsServer: could not resolve UGV/UAV mobility modules; "
                       "distance metric will be zero." << omnetpp::endl;

        // Subscribe to radio delivery signals on the UGV radio module.
        // Path relative to the network: e.g. "ugv.wlan[0].radio"
        std::string radioPath = ugvModulePath + "." + radioModulePath;
        omnetpp::cModule* radioMod = getModuleByPath(radioPath.c_str());
        if (radioMod) {
            radioMod->subscribe(packetSentToUpperSignal, this);
            radioMod->subscribe(packetDroppedSignal, this);
            radioMod->subscribe(droppedPacketSignal, this);
            EV_INFO << "OmnetMetricsServer: subscribed to delivery signals on "
                    << radioPath << omnetpp::endl;
        }
        else {
            EV_WARN << "OmnetMetricsServer: radio module not found at path '"
                    << radioPath << "'; PER will report 0." << omnetpp::endl;
        }

        getSimulation()->getSystemModule()->subscribe(loraAppPacketSentSignal, this);
    }
}

void OmnetMetricsServer::handleMessage(cMessage* msg)
{
    if (msg == metricsTimer) {
        tryAcceptClient();
        sendMetricsLine();
        scheduleAt(omnetpp::simTime() + updateInterval, metricsTimer);
    }
    else {
        delete msg;
    }
}

void OmnetMetricsServer::finish()
{
    closeClientSocket();
    closeServerSocket();
}

// ── Signal listener ───────────────────────────────────────────────────────

void OmnetMetricsServer::receiveSignal(omnetpp::cComponent* /*src*/,
                                       omnetpp::simsignal_t  signal,
                                       omnetpp::cObject*     obj,
                                       omnetpp::cObject*     /*details*/)
{
    if (signal == packetSentToUpperSignal)
        handleDeliveredPacket(dynamic_cast<inet::Packet*>(obj));
    else if (signal == packetDroppedSignal || signal == droppedPacketSignal)
        handleDroppedPacket();
}

void OmnetMetricsServer::receiveSignal(omnetpp::cComponent* /*src*/,
                                       omnetpp::simsignal_t  signal,
                                       omnetpp::intval_t     /*value*/,
                                       omnetpp::cObject*     /*details*/)
{
    if (signal == loraAppPacketSentSignal)
        handlePacketSentByApp();
    else if (signal == packetDroppedSignal || signal == droppedPacketSignal)
        handleDroppedPacket();
}

// ── Socket management ─────────────────────────────────────────────────────

void OmnetMetricsServer::startServer()
{
    if (initsocketlibonce() != 0)
        throw cRuntimeError("OmnetMetricsServer: could not init socket library");

    SOCKET* sockPtr = new SOCKET();
    *sockPtr = ::socket(AF_INET, SOCK_STREAM, 0);
    if (*sockPtr == INVALID_SOCKET) {
        delete sockPtr;
        throw cRuntimeError("OmnetMetricsServer: could not create server socket");
    }

    int reuse = 1;
    ::setsockopt(*sockPtr, SOL_SOCKET, SO_REUSEADDR,
                 reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(static_cast<uint16_t>(port));

    if (::bind(*sockPtr, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        closesocket(*sockPtr);
        delete sockPtr;
        throw cRuntimeError("OmnetMetricsServer: bind failed on port %d: %s",
                            port, strerror(sock_errno()));
    }

    if (::listen(*sockPtr, 4) < 0) {
        closesocket(*sockPtr);
        delete sockPtr;
        throw cRuntimeError("OmnetMetricsServer: listen failed on port %d: %s",
                            port, strerror(sock_errno()));
    }

    serverSockPtr = sockPtr;
    EV_INFO << "OmnetMetricsServer: listening on port " << port << omnetpp::endl;
}

void OmnetMetricsServer::tryAcceptClient()
{
    if (!serverSockPtr)
        return;

    SOCKET serverSock = sockFromPtr(serverSockPtr);

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(serverSock, &readfds);
    timeval tv = {0, 0}; // non-blocking poll

    int ret = ::select(static_cast<int>(serverSock) + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0)
        return;

    sockaddr_in clientAddr;
    socklen_t   clientLen = sizeof(clientAddr);
    SOCKET clientSock = ::accept(serverSock,
                                 reinterpret_cast<sockaddr*>(&clientAddr),
                                 &clientLen);
    if (clientSock == INVALID_SOCKET)
        return;

    if (clientSockPtr) {
        EV_INFO << "OmnetMetricsServer: replacing existing client" << omnetpp::endl;
        closeClientSocket();
    }

    int noDelay = 1;
    ::setsockopt(clientSock, IPPROTO_TCP, TCP_NODELAY,
                 reinterpret_cast<const char*>(&noDelay), sizeof(noDelay));

    clientSockPtr = new SOCKET(clientSock);
    EV_INFO << "OmnetMetricsServer: client connected" << omnetpp::endl;
}

void OmnetMetricsServer::sendMetricsLine()
{
    if (!clientSockPtr)
        return;

    double dist       = usePathLossFallback ? computeDistance() : 0.0;
    double rssi       = currentRssiDbm(dist);
    double snir       = currentSnirDb(dist);
    double per        = computePer();
    double radioDist  = publishRadioDistanceEstimate ? computeRadioDistance(rssi)
                                                     : std::numeric_limits<double>::quiet_NaN();
    double pdr        = computePdr();

    char buf[384];
    int n = snprintf(buf, sizeof(buf), "%.3f %.2f %.2f %.4f %.3f %.4f %.6f %.6f\n",
                     omnetpp::simTime().dbl(), rssi, snir, per, radioDist,
                     pdr, lastLatencyS, lastJitterS);
    if (n <= 0)
        return;

    SOCKET clientSock = sockFromPtr(clientSockPtr);
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32) || defined(__CYGWIN__) || defined(_WIN64)
    ssize_t sent = ::send(clientSock, buf, static_cast<size_t>(n), 0);
#else
    ssize_t sent = ::send(clientSock, buf, static_cast<size_t>(n), MSG_NOSIGNAL);
#endif
    if (sent <= 0) {
        EV_INFO << "OmnetMetricsServer: client disconnected" << omnetpp::endl;
        closeClientSocket();
    }
}

void OmnetMetricsServer::closeClientSocket()
{
    if (clientSockPtr) {
        closesocket(sockFromPtr(clientSockPtr));
        delete static_cast<SOCKET*>(clientSockPtr);
        clientSockPtr = nullptr;
    }
}

void OmnetMetricsServer::closeServerSocket()
{
    if (serverSockPtr) {
        closesocket(sockFromPtr(serverSockPtr));
        delete static_cast<SOCKET*>(serverSockPtr);
        serverSockPtr = nullptr;
    }
}

// ── Metric computation ────────────────────────────────────────────────────

double OmnetMetricsServer::computeDistance() const
{
    if (!ugvMobility || !uavMobility)
        return 0.0;

    const inet::Coord& ugvPos = ugvMobility->getCurrentPosition();
    const inet::Coord& uavPos = uavMobility->getCurrentPosition();
    double dx = uavPos.x - ugvPos.x;
    double dy = uavPos.y - ugvPos.y;
    double dz = uavPos.z - ugvPos.z;
    double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
    return (d < 0.1) ? 0.1 : d; // avoid log(0)
}

double OmnetMetricsServer::computeRssiDbm(double distanceM) const
{
    // Free-space path loss (dB):
    //   FSPL = 20*log10(d) + 20*log10(f) + 20*log10(4π/c)
    //        = 20*log10(d) + 20*log10(f) - 147.55  (d in metres, f in Hz)
    double fspl = 20.0 * std::log10(distanceM)
                + 20.0 * std::log10(carrierFreqHz)
                - 147.55;
    return txPowerDbm - fspl;
}

double OmnetMetricsServer::computeSnirDb(double distanceM) const
{
    double receivedPowerDbm = computeRssiDbm(distanceM);
    return receivedPowerDbm - noisePowerDbm;
}

double OmnetMetricsServer::computePer() const
{
    if (perWindow.empty())
        return std::numeric_limits<double>::quiet_NaN();

    int dropped = 0;
    for (bool received : perWindow)
        if (!received)
            ++dropped;

    return static_cast<double>(dropped) / static_cast<double>(perWindow.size());
}

double OmnetMetricsServer::computePdr() const
{
    if (perWindow.empty())
        return std::numeric_limits<double>::quiet_NaN();

    int delivered = 0;
    for (bool received : perWindow)
        if (received)
            ++delivered;

    return static_cast<double>(delivered) / static_cast<double>(perWindow.size());
}

double OmnetMetricsServer::computeRadioDistance(double rssiDbm) const
{
    if (!publishRadioDistanceEstimate || !std::isfinite(rssiDbm))
        return std::numeric_limits<double>::quiet_NaN();

    if (radioDistanceModel == "fspl") {
        // Invert free-space path loss to estimate distance from RSSI:
        //   RSSI = TxPower - (20*log10(d) + 20*log10(f) - 147.55)
        double exp = (txPowerDbm - rssiDbm - 20.0 * std::log10(carrierFreqHz) + 147.55) / 20.0;
        double d   = std::pow(10.0, exp);
        return (d < 0.1) ? 0.1 : d;
    }

    if (radioDistanceModel == "loraLogNormal") {
        if (loraPathLossD0M <= 0.0 || loraPathLossGamma <= 0.0)
            return std::numeric_limits<double>::quiet_NaN();

        // Invert FLORA LoRaLogNormalShadowing without using Gazebo pose:
        //   PL = PLd0 + 10*gamma*log10(d/d0) + shadowing
        // This returns the model-implied distance from actual received RSSI.
        double pathLossDb = txPowerDbm - rssiDbm;
        double exp = (pathLossDb - loraPathLossPLd0Db) / (10.0 * loraPathLossGamma);
        double d = loraPathLossD0M * std::pow(10.0, exp);
        return (d < 0.1) ? 0.1 : d;
    }

    return std::numeric_limits<double>::quiet_NaN();
}

double OmnetMetricsServer::currentRssiDbm(double distanceM) const
{
    if (std::isfinite(lastRssiDbm))
        return lastRssiDbm;
    return usePathLossFallback ? computeRssiDbm(distanceM)
                               : std::numeric_limits<double>::quiet_NaN();
}

double OmnetMetricsServer::currentSnirDb(double distanceM) const
{
    if (std::isfinite(lastSnirDb))
        return lastSnirDb;
    return usePathLossFallback ? computeSnirDb(distanceM)
                               : std::numeric_limits<double>::quiet_NaN();
}

void OmnetMetricsServer::markReception(bool received)
{
    perWindow.push_back(received);
    if ((int)perWindow.size() > PER_WINDOW)
        perWindow.pop_front();
}

void OmnetMetricsServer::handlePacketSentByApp()
{
    pendingSendTimes.push_back(omnetpp::simTime());
    while (pendingSendTimes.size() > 1000)
        pendingSendTimes.pop_front();

    if (omnetpp::simTime() >= getSimulation()->getWarmupPeriod())
        sentPackets++;
}

void OmnetMetricsServer::handleDeliveredPacket(inet::Packet* packet)
{
    if (omnetpp::simTime() >= getSimulation()->getWarmupPeriod()) {
        deliveredPackets++;
        markReception(true);
    }

    updatePhyMetrics(packet);
    updateLatency();
}

void OmnetMetricsServer::handleDroppedPacket()
{
    if (omnetpp::simTime() >= getSimulation()->getWarmupPeriod()) {
        droppedPackets++;
        markReception(false);
    }

    if (!pendingSendTimes.empty())
        pendingSendTimes.pop_front();
}

void OmnetMetricsServer::updatePhyMetrics(inet::Packet* packet)
{
    if (!packet)
        return;

    if (auto signalPowerInd = packet->findTag<inet::SignalPowerInd>()) {
        double mW = signalPowerInd->getPower().get() * 1000.0;
        if (mW > 0.0 && std::isfinite(mW))
            lastRssiDbm = inet::math::mW2dBmW(mW);
    }

    if (auto snirInd = packet->findTag<inet::SnirInd>()) {
        double snir = snirInd->getMinimumSnir();
        if (snir > 0.0 && std::isfinite(snir))
            lastSnirDb = inet::math::fraction2dB(snir);
    }

    if (auto errorRateInd = packet->findTag<inet::ErrorRateInd>()) {
        double packetErrorRate = errorRateInd->getPacketErrorRate();
        if (std::isfinite(packetErrorRate))
            lastPhyPacketErrorRate = packetErrorRate;
    }
}

void OmnetMetricsServer::updateLatency()
{
    if (pendingSendTimes.empty())
        return;

    double latency = (omnetpp::simTime() - pendingSendTimes.front()).dbl();
    pendingSendTimes.pop_front();

    if (latency < 0.0 || !std::isfinite(latency))
        return;

    if (std::isfinite(previousLatencyS))
        lastJitterS = std::fabs(latency - previousLatencyS);
    previousLatencyS = latency;
    lastLatencyS = latency;
}
