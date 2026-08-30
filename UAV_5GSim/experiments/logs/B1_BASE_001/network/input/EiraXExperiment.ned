import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.node.ethernet.Eth10G;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import simu5g.common.binder.Binder;
import simu5g.common.carrierAggregation.CarrierAggregation;
import simu5g.nodes.Upf;
import simu5g.nodes.NR.gNodeB;
import simu5g.nodes.NR.NRUe;
import simu5g.world.radio.LteChannelControl;
network EiraXExperiment {
    parameters: int numUe=default(1);
    submodules:
        channelControl: LteChannelControl;
        routingRecorder: RoutingTableRecorder;
        configurator: Ipv4NetworkConfigurator;
        binder: Binder;
        carrierAggregation: CarrierAggregation;
        missionBackend: StandardHost;
        edgeNode: StandardHost;
        router: Router;
        upf: Upf;
        iUpf: Upf;
        gnb: gNodeB;
        ue[numUe]: NRUe;
    connections:
        missionBackend.pppg++ <--> Eth10G <--> router.pppg++;
        edgeNode.pppg++ <--> Eth10G <--> router.pppg++;
        router.pppg++ <--> Eth10G <--> upf.filterGate;
        upf.pppg++ <--> Eth10G <--> iUpf.pppg++;
        iUpf.pppg++ <--> Eth10G <--> gnb.ppp;
}
