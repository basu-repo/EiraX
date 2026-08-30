
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
network EiraXOneUas {
 parameters: int numUe = default(1); @display("i=block/network2;bgb=1000,700");
 submodules:
  channelControl: LteChannelControl; routingRecorder: RoutingTableRecorder;
  configurator: Ipv4NetworkConfigurator; binder: Binder; carrierAggregation: CarrierAggregation;
  missionBackend: StandardHost { @display("p=120,100;i=device/server"); }
  edgeNode: StandardHost { @display("p=280,100;i=device/server"); }
  router: Router { @display("p=440,100;i=device/smallrouter"); }
  upf: Upf { @display("p=590,100"); } iUpf: Upf { @display("p=740,100"); }
  gnb: gNodeB { @display("p=740,280;is=vl"); } ue[numUe]: NRUe { @display("p=740,470"); }
 connections:
  missionBackend.pppg++ <--> Eth10G <--> router.pppg++;
  edgeNode.pppg++ <--> Eth10G <--> router.pppg++;
  router.pppg++ <--> Eth10G <--> upf.filterGate;
  upf.pppg++ <--> Eth10G <--> iUpf.pppg++;
  iUpf.pppg++ <--> Eth10G <--> gnb.ppp;
}
