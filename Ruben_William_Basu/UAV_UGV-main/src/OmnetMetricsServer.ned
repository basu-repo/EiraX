package uav_ugv;

//
// OmnetMetricsServer — live OMNeT→ROS2 network metrics bridge.
//
// Listens on a TCP port and periodically sends one ASCII snapshot line
// to a connected client (e.g. the ROS2 omnet_metrics_bridge node):
//
//   <simtime_s> <rssi_dbm> <snir_db> <per>
//   <radio_distance_m> <pdr> <latency_s> <jitter_s>
//
// RSSI and SNIR are read from PHY tags on received packets when available.
// PER/PDR/latency/jitter are estimated from radio/app events.
//
simple OmnetMetricsServer
{
    parameters:
        int    port           = default(5556);
        double updateInterval @unit(s) = default(0.1s);
        string ugvModule      = default("^.ugv");
        string uavModule      = default("^.uav");
        // Path to the UGV radio module, relative to the ugvModule.
        // Change to "LoRaNic.radio" for LoRa configs.
        string radioModulePath = default("wlan[0].radio");
        // Transmit power of the UAV radio (dBm).
        double txPowerDbm     = default(13.0);    // ~20 mW (802.11g)
        // Carrier frequency (Hz) — must match the radio medium config.
        double carrierFreqHz  = default(2.4e9);   // 2.4 GHz WiFi default
        // Thermal + receiver noise floor (dBm).
        double noisePowerDbm  = default(-90.0);
        // Legacy fallback for configs whose radio stack does not attach PHY tags.
        bool   usePathLossFallback = default(false);
        // Publish RSSI-inverted distance estimate in field 6.
        bool   publishRadioDistanceEstimate = default(false);
        // "none", "fspl", or "loraLogNormal".
        string radioDistanceModel = default("none");
        // Parameters for inverting FLORA LoRaLogNormalShadowing:
        // PL = PLd0 + 10*gamma*log10(d/d0) + shadowing.
        double loraPathLossD0 @unit(m) = default(40m);
        double loraPathLossGamma = default(2.08);
        double loraPathLossPLd0Db = default(127.41);
        @display("i=block/server");
        @class(OmnetMetricsServer);
}
