// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <deque>
#include <limits>
#include <string>

#include <omnetpp.h>

#include "inet/common/geometry/common/Coord.h"
#include "inet/mobility/contract/IMobility.h"

namespace inet {
class Packet;
}

/**
 * OmnetMetricsServer — live OMNeT→ROS2 network metrics bridge.
 *
 * Listens on a TCP port and, at each update interval, sends a single
 * ASCII snapshot line to any connected client:
 *
 *   <simtime_s> <rssi_dbm> <snir_db> <per>
 *   <radio_distance_m> <pdr> <latency_s> <jitter_s>
 *
 * RSSI and SNIR are read from packet-level PHY tags when the radio stack
 * provides them. Field 6 is an RSSI-derived radio distance estimate; for LoRa
 * it inverts FLORA's LoRaLogNormalShadowing model and does not read Gazebo pose.
 *
 * The server accepts exactly one client at a time.  If the client
 * disconnects it is replaced by the next one that connects.
 */
class OmnetMetricsServer : public omnetpp::cSimpleModule, public omnetpp::cListener
{
  protected:
    // ── parameters ────────────────────────────────────────────────────
    int         port            = 5556;
    double      updateInterval  = 0.1;   // seconds
    std::string ugvModulePath;           // e.g. "ugv"
    std::string uavModulePath;           // e.g. "uav"
    std::string radioModulePath;         // e.g. "wlan[0].radio"  (relative to ugv)
    double      txPowerDbm      = -4.0;  // 20 mW default for IEEE 802.11g
    double      carrierFreqHz   = 2.4e9; // 2.4 GHz
    double      noisePowerDbm   = -90.0; // thermal noise floor
    bool        usePathLossFallback = false;
    bool        publishRadioDistanceEstimate = false;
    std::string radioDistanceModel = "none";
    double      loraPathLossD0M = 40.0;
    double      loraPathLossGamma = 2.08;
    double      loraPathLossPLd0Db = 127.41;

    // ── runtime state ─────────────────────────────────────────────────
    inet::IMobility* ugvMobility = nullptr;
    inet::IMobility* uavMobility = nullptr;

    // Packet delivery sliding window (true=received, false=dropped)
    static constexpr int PER_WINDOW = 30;
    std::deque<bool> perWindow;
    std::deque<omnetpp::simtime_t> pendingSendTimes;

    long sentPackets = 0;
    long deliveredPackets = 0;
    long droppedPackets = 0;

    double lastRssiDbm = std::numeric_limits<double>::quiet_NaN();
    double lastSnirDb = std::numeric_limits<double>::quiet_NaN();
    double lastPhyPacketErrorRate = std::numeric_limits<double>::quiet_NaN();
    double lastLatencyS = std::numeric_limits<double>::quiet_NaN();
    double previousLatencyS = std::numeric_limits<double>::quiet_NaN();
    double lastJitterS = std::numeric_limits<double>::quiet_NaN();

    // Timer
    omnetpp::cMessage* metricsTimer = nullptr;

    // TCP server / client sockets (opaque pointers to SOCKET)
    void* serverSockPtr = nullptr;
    void* clientSockPtr = nullptr;

    // Cached signal IDs
    static const omnetpp::simsignal_t packetSentToUpperSignal;
    static const omnetpp::simsignal_t packetDroppedSignal;
    static const omnetpp::simsignal_t droppedPacketSignal;
    static const omnetpp::simsignal_t loraAppPacketSentSignal;

  protected:
    // ── OMNeT++ lifecycle ──────────────────────────────────────────────
    virtual int  numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void finish() override;

    // ── cListener ─────────────────────────────────────────────────────
    // Receives packetSentToUpper / packetDropped (cObject* variant)
    virtual void receiveSignal(omnetpp::cComponent* src,
                               omnetpp::simsignal_t  signal,
                               omnetpp::cObject*     obj,
                               omnetpp::cObject*     details) override;
    virtual void receiveSignal(omnetpp::cComponent* src,
                               omnetpp::simsignal_t  signal,
                               omnetpp::intval_t     value,
                               omnetpp::cObject*     details) override;

  private:
    // ── socket helpers ────────────────────────────────────────────────
    void startServer();
    void tryAcceptClient();
    void sendMetricsLine();
    void closeClientSocket();
    void closeServerSocket();

    // ── metric computation ────────────────────────────────────────────
    double computeDistance() const;
    double computeRssiDbm(double distanceM) const;
    double computeSnirDb(double distanceM) const;
    double computePer() const;
    double computePdr() const;
    double computeRadioDistance(double rssiDbm) const;
    double currentRssiDbm(double distanceM) const;
    double currentSnirDb(double distanceM) const;
    void markReception(bool received);
    void handlePacketSentByApp();
    void handleDeliveredPacket(inet::Packet* packet);
    void handleDroppedPacket();
    void updatePhyMetrics(inet::Packet* packet);
    void updateLatency();
};
