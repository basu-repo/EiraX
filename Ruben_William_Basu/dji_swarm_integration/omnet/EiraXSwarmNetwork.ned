import inet.common.geometry.common.IGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.common.IntegratedVisualizer;
import uav_ugv.GazeboPositionScheduler;
import uav_ugv.OmnetMetricsServer;
import uav_ugv.UAV;
import uav_ugv.UGV;

// INET-only swarm network. No Simu5G or FLORA dependency is required.
network EiraXSwarmNetwork
{
    parameters:
        int numUavs = default(1);
        bool enablePoseBridge = default(true);
        bool enableMetrics = default(true);
        @display("bgb=900,600");
    submodules:
        gazeboScheduler: GazeboPositionScheduler if enablePoseBridge {
            @display("p=60,510");
        }
        metricsServer[numUavs]: OmnetMetricsServer if enableMetrics {
            @display("p=180,510,row,130");
        }
        visualizer: IntegratedVisualizer {
            @display("p=60,430");
        }
        coordinateSystem: <default("SimpleGeographicCoordinateSystem")> like IGeographicCoordinateSystem {
            @display("p=220,430");
        }
        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=380,430");
        }
        physicalEnvironment: PhysicalEnvironment {
            @display("p=540,430");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=700,430");
        }
        ugv: UGV {
            @display("p=450,110;i=device/carright");
        }
        uav[numUavs]: UAV {
            @display("p=220,240,row,220;i=device/drone");
        }
}
