#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/buildings-module.h"
#include "ns3/netanim-module.h" 
#include <fstream>
#include <map>
#include <deque>
#include <string>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ScenarioUavMecAdvanced");

// ========== biến toàn cục và struct ==========
std::ofstream rsrpFile, sinrFile, throughputFile, handoverFile, positionFile, cellIdFile, handoverQualityFile, uavPositionFile, mecOffloadFile, hoTraceFile, flowStatsFile;

uint32_t handoverCount = 0;
uint32_t handoverStartCount = 0;
std::map<uint32_t, uint64_t> totalRxBytes;
std::map<uint32_t, double> lastThroughput;

// Map theo dõi UE đang ở Cell nào
std::map<uint32_t, uint16_t> ueCurrentCellMap;
std::map<uint32_t, uint16_t> uePreviousMecCellMap; 

struct HandoverEvent {
    double time; uint64_t imsi; uint16_t sourceCellId; uint16_t targetCellId;
    double rsrpBefore; double rsrpAfter; double throughputBefore; double throughputAfter;
};

std::map<uint64_t, std::deque<uint16_t>> ueHandoverHistory; 
std::map<uint64_t, HandoverEvent> lastHandoverEvent;
uint32_t pingPongHandoverCount = 0;

// Visual Globals
AnimationInterface *pAnim = 0; 
NodeContainer globalUeNodes;

struct OffloadTask {
    uint32_t taskId; double latency; bool offloaded; double energy;
};
std::vector<OffloadTask> offloadTasks;
uint32_t taskCounter = 0;

// ============== VISUAL HELPER ======================
void UpdateUeColor(uint32_t ueIndex, uint16_t cellId) {
    if (pAnim && ueIndex < globalUeNodes.GetN()) {
        uint8_t r=0, g=0, b=0;
        if (cellId == 1) { r=255; g=0; b=0; }      
        else if (cellId == 2) { r=0; g=255; b=0; } 
        else if (cellId == 3) { r=128; g=0; b=128; }
        else { r=100; g=100; b=100; } // Unknown cell
        pAnim->UpdateNodeColor(globalUeNodes.Get(ueIndex), r, g, b);
        pAnim->UpdateNodeDescription(globalUeNodes.Get(ueIndex), "UE-" + std::to_string(ueIndex+1) + " (C" + std::to_string(cellId) + ")");
    }
}

// ==================== MEC LOGIC  ====================
void GenerateMecTask(uint32_t ueId) {
    double time = Simulator::Now().GetSeconds();
    double dataSize = 0.1; // Mbits
    double cpuCycles = 2000.0; // Megacycles
    
    // Lấy throughput hiện tại
    double thpt = lastThroughput[ueId];
    if (thpt < 0.1) thpt = 5.0; 
    
    // Tính toán thời gian truyền và xử lý cơ bản
    double txTime = (dataSize * 8.0) / thpt;
    double uavCpuSpeed = 5000.0; // Cycles/s
    double procTime = cpuCycles / uavCpuSpeed;
    
    // Tính toán Migration Penalty
    double migrationPenalty = 0.0;
    uint16_t currentCell = ueCurrentCellMap[ueId];
    uint16_t prevCell = uePreviousMecCellMap[ueId];

    // Nếu Cell thay đổi so với lần tính trước -> Đã có Handover -> Phạt thời gian di trú
    if (prevCell != 0 && currentCell != 0 && currentCell != prevCell) {
        migrationPenalty = 0.050; // Phạt 50ms cho việc di chuyển container giữa các UAV
        // Ghi log sự kiện di trú
        std::cout << "   [MEC MIGRATION] Time: " << time << "s | UE" << ueId 
                  << " migrating data C" << prevCell << " -> C" << currentCell 
                  << " (+50ms delay)" << std::endl;
    }
    
    // Cập nhật lại Cell đã xử lý
    uePreviousMecCellMap[ueId] = currentCell;

    // Tổng độ trễ Offload
    double totalLatency = txTime + procTime + migrationPenalty;
    
    // Độ trễ xử lý tại chỗ (Local)
    double localTime = cpuCycles / 1000.0; // Local CPU yếu hơn (1000)
    
    // Quyết định Offload
    bool offloaded = totalLatency < localTime;
    double finalLatency = offloaded ? totalLatency : localTime;
    
    offloadTasks.push_back({taskCounter++, finalLatency, offloaded, 0.0});
    
    // Ghi file CSV: Thêm MigrationPenalty 
    mecOffloadFile << time << "," << ueId << "," << taskCounter << ","
                   << (offloaded ? "UAV-MEC" : "Local") << ","
                   << finalLatency << "," << thpt << "," << migrationPenalty << std::endl;
}

// ==================== HANDOVER CALLBACKS ====================
bool IsPingPongHandover(uint64_t imsi, uint16_t targetCellId) {
    if (ueHandoverHistory[imsi].size() < 2) return false;
    auto& history = ueHandoverHistory[imsi];
    return (history[history.size()-1] == targetCellId || history[history.size()-2] == targetCellId);
}

void NotifyHandoverStartUe(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId) {   
    handoverStartCount++;
    HandoverEvent event; event.time = Simulator::Now().GetSeconds();
    event.imsi = imsi; event.sourceCellId = cellId; event.targetCellId = targetCellId;
    event.throughputBefore = lastThroughput[imsi];
    lastHandoverEvent[imsi] = event;
}

void NotifyHandoverEndOkUe(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti) {
    handoverCount++;
    double time = Simulator::Now().GetSeconds();
    
    // Cập nhật bản đồ vị trí ngay lập tức để MEC biết
    ueCurrentCellMap[imsi] = cellId; 

    if (IsPingPongHandover(imsi, cellId)) pingPongHandoverCount++;
    
    ueHandoverHistory[imsi].push_back(cellId);
    if (ueHandoverHistory[imsi].size() > 3) ueHandoverHistory[imsi].pop_front();
    
    if (lastHandoverEvent.find(imsi) != lastHandoverEvent.end()) {
        HandoverEvent& ev = lastHandoverEvent[imsi];
        double duration = time - ev.time;
        double degradation = 0.0;
        if (ev.throughputBefore > 0) degradation = (ev.throughputBefore - lastThroughput[imsi])/ev.throughputBefore*100.0;
        
        handoverQualityFile << time << "," << imsi << "," << ev.sourceCellId << ","
                           << ev.targetCellId << ",No," << duration << "," 
                           << ev.throughputBefore << "," << lastThroughput[imsi] << "," << degradation << std::endl;
        
        hoTraceFile << time << "," << imsi << "," << ev.sourceCellId << "," << ev.targetCellId << std::endl;
        
        std::cout << "   [HO SUCCESS] Time: " << std::fixed << std::setprecision(2) << time 
                  << "s | UE" << imsi << " switched: Cell " << ev.sourceCellId 
                  << " --> Cell " << ev.targetCellId << std::endl;
    }
    handoverFile << time << "," << imsi << "," << cellId << "," << handoverCount << ",0" << std::endl;
    
    UpdateUeColor(imsi - 1, cellId);
}

void RecvMeasurementReportCallback(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti, LteRrcSap::MeasurementReport report) {
    double time = Simulator::Now().GetSeconds();
    if (report.measResults.haveMeasResultNeighCells) {
        for (auto it = report.measResults.measResultListEutra.begin(); it != report.measResults.measResultListEutra.end(); ++it) {
            if (it->haveRsrpResult) {
                double rsrp = -140.0 + (double)it->rsrpResult;
                rsrpFile << time << "," << imsi << "," << it->physCellId << "," << rsrp << std::endl;
            }
        }
    }
}

void ReportRsrp(uint64_t imsi, uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t ccId) {
    rsrpFile << Simulator::Now().GetSeconds() << "," << imsi << "," << cellId << "," << rsrp << std::endl;
    sinrFile << Simulator::Now().GetSeconds() << "," << imsi << "," << sinr << std::endl;
}

void NotifyConnectionEstablished(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti) {
    cellIdFile << Simulator::Now().GetSeconds() << "," << imsi << "," << cellId << std::endl;
    // Khởi tạo vị trí ban đầu
    ueCurrentCellMap[imsi] = cellId;
    uePreviousMecCellMap[imsi] = cellId; 
    UpdateUeColor(imsi - 1, cellId);
}

void LogPositions(NodeContainer ues, NodeContainer uavs) {
    double t = Simulator::Now().GetSeconds();
    for (uint32_t i=0; i<ues.GetN(); ++i) {
        Vector p = ues.Get(i)->GetObject<MobilityModel>()->GetPosition();
        positionFile << t << ",UE" << (i+1) << "," << p.x << "," << p.y << "," << p.z << std::endl;
    }
    for (uint32_t i=0; i<uavs.GetN(); ++i) {
        Vector p = uavs.Get(i)->GetObject<MobilityModel>()->GetPosition();
        uavPositionFile << t << ",UAV" << (i+1) << "," << p.x << "," << p.y << "," << p.z << std::endl;
    }
}

void CalculateThroughput(Ptr<PacketSink> sink, uint32_t ueId, double window) {
    double time = Simulator::Now().GetSeconds();
    uint64_t rx = sink->GetTotalRx();
    if (totalRxBytes.find(ueId) == totalRxBytes.end()) totalRxBytes[ueId] = 0;
    double thpt = (rx - totalRxBytes[ueId]) * 8.0 / 1e6 / window;
    throughputFile << time << "," << ueId << "," << thpt << std::endl;
    totalRxBytes[ueId] = rx;
    lastThroughput[ueId] = thpt;
}

// ==================== MAIN ====================
int main(int argc, char *argv[])
{
    double simTime = 99.0; 
    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.Parse(argc, argv);
    
    RngSeedManager::SetSeed(12345); RngSeedManager::SetRun(1);
    
    std::cout << "SCENARIO: UAV-MEC Handover with Migration Penalty & FlowMonitor" << std::endl;
    
    // Open CSV Files
    rsrpFile.open("scenario1_final_rsrp.csv"); rsrpFile << "Time,IMSI,CellId,RSRP" << std::endl;
    sinrFile.open("scenario1_final_sinr.csv"); sinrFile << "Time,IMSI,SINR" << std::endl;
    throughputFile.open("scenario1_final_throughput.csv"); throughputFile << "Time,UE_ID,Throughput_Mbps" << std::endl;
    handoverFile.open("scenario1_final_handover.csv"); handoverFile << "Time,IMSI,TargetCellId,HandoverCount,PingPong" << std::endl;
    positionFile.open("scenario1_final_ue_position.csv"); positionFile << "Time,NodeName,X,Y,Z" << std::endl;
    cellIdFile.open("scenario1_final_cellid.csv"); cellIdFile << "Time,IMSI,CellId" << std::endl;
    handoverQualityFile.open("scenario1_final_handover_quality.csv"); handoverQualityFile << "Time,IMSI,Source,Target,PingPong,Duration,TpBefore,TpAfter,Degradation" << std::endl;
    uavPositionFile.open("scenario1_final_uav_position.csv"); uavPositionFile << "Time,NodeName,X,Y,Z" << std::endl;
    mecOffloadFile.open("scenario1_final_mec_offload.csv"); mecOffloadFile << "Time,UE_ID,TaskID,Type,Latency,Throughput,MigrationPenalty" << std::endl;
    hoTraceFile.open("handover_trace.csv"); hoTraceFile << "Time(s),UE_ID,From_Cell,To_Cell" << std::endl;
    flowStatsFile.open("scenario1_final_flow_stats.csv"); flowStatsFile << "FlowID,Src,Dst,TxPkts,RxPkts,LostPkts,LossRate,Delay(ms),Jitter(ms)" << std::endl;

    // LTE Configuration
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetAttribute("UseIdealRrc", BooleanValue(false));
    
    // --- TUNING HANDOVER SENSITIVITY ---
    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0.1)); 
    lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(20))); 
    
    //lteHelper->SetAttribute("PathlossModel", StringValue("ns3::FriisPropagationLossModel"));
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::LogDistancePropagationLossModel"));
    
    // Cấu hình môi trường truyền sóng (Exponent = 3.0 tương đương đô thị)
    lteHelper->SetPathlossModelAttribute("Exponent", DoubleValue(3.0)); 
    lteHelper->SetPathlossModelAttribute("ReferenceLoss", DoubleValue(46.67)); // Chuẩn LTE 2GHz
    
    // Network Setup
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer; remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet; internet.Install(remoteHostContainer);
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h; ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevices);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>())->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    
    MobilityHelper coreMobility;
    Ptr<ListPositionAllocator> corePos = CreateObject<ListPositionAllocator>();
    
    // 1. Đặt Remote Host (Server) ở góc trên bên trái
    corePos->Add(Vector(10, 250, 0)); 
    
    // 2. Đặt PGW (Gateway) thấp hơn Server một chút
    corePos->Add(Vector(10, 200, 0));
    
    // 3. SGW (Serving Gateway) - do EPC Helper tự tạo, là Node số 1
    corePos->Add(Vector(30, 180, 0));
    
    // 4. MME (Quản lý di động) -  do EPC Helper tự tạo, là Node số 2
    corePos->Add(Vector(10, 160, 0));

    coreMobility.SetPositionAllocator(corePos);
    coreMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    // Cài đặt vị trí cho Remote Host 
    coreMobility.Install(remoteHostContainer);
    
    // Cài đặt vị trí cho PGW
    NodeContainer pgwContainer;
    pgwContainer.Add(pgw);
    coreMobility.Install(pgwContainer);
    Ptr<Node> sgw = epcHelper->GetSgwNode();
    NodeContainer sgwContainer;
    sgwContainer.Add(sgw);
    coreMobility.Install(sgwContainer);
    
    // MME là Node có ID = 2 
    Ptr<Node> mme = NodeList::GetNode(2); 
    NodeContainer mmeContainer;
    mmeContainer.Add(mme);
    // lấy vị trí thứ 4 trong ListPositionAllocator (Vector 10, 160, 0)
    coreMobility.Install(mmeContainer);
    
    // Nodes
    NodeContainer uavEnbNodes; uavEnbNodes.Create(3);
    globalUeNodes.Create(3); 
    
    // Mobility: UAVs
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::WaypointMobilityModel");
    mob.Install(uavEnbNodes); 
    
    // UAV 1: Bay tuần tra quanh khu vực (80, 80) với bán kính 40m
    Ptr<WaypointMobilityModel> uav1 = uavEnbNodes.Get(0)->GetObject<WaypointMobilityModel>();
    uav1->AddWaypoint(Waypoint(Seconds(0), Vector(80, 80, 30)));
    uav1->AddWaypoint(Waypoint(Seconds(25), Vector(120, 80, 30))); 
    uav1->AddWaypoint(Waypoint(Seconds(50), Vector(80, 120, 30)));
    uav1->AddWaypoint(Waypoint(Seconds(75), Vector(40, 80, 30)));  
    uav1->AddWaypoint(Waypoint(Seconds(99), Vector(80, 80, 30)));  

    // UAV 2: Bay quanh khu vực (220, 80)
    Ptr<WaypointMobilityModel> uav2 = uavEnbNodes.Get(1)->GetObject<WaypointMobilityModel>();
    uav2->AddWaypoint(Waypoint(Seconds(0), Vector(220, 80, 30)));
    uav2->AddWaypoint(Waypoint(Seconds(30), Vector(220, 40, 30))); 
    uav2->AddWaypoint(Waypoint(Seconds(60), Vector(260, 80, 30))); 
    uav2->AddWaypoint(Waypoint(Seconds(99), Vector(220, 80, 30)));

    // UAV 3: Bay quanh khu vực (150, 220)
    Ptr<WaypointMobilityModel> uav3 = uavEnbNodes.Get(2)->GetObject<WaypointMobilityModel>();
    uav3->AddWaypoint(Waypoint(Seconds(0), Vector(150, 220, 30)));
    uav3->AddWaypoint(Waypoint(Seconds(40), Vector(190, 220, 30))); 
    uav3->AddWaypoint(Waypoint(Seconds(99), Vector(150, 220, 30)));
    
    // Mobility: UEs
    // --- SETUP UE 1 ---
    MobilityHelper ue1Mob;
    Ptr<ListPositionAllocator> ue1Pos = CreateObject<ListPositionAllocator>();
    ue1Pos->Add(Vector(60, 99, 1.5)); // Vị trí khởi tạo nằm trong Bounds
    ue1Mob.SetPositionAllocator(ue1Pos);
    ue1Mob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                            "Bounds", RectangleValue(Rectangle(50, 100, 50, 100)), // Vùng đi quanh UAV 1
                            "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"),
                            "Mode", StringValue("Time"));
    ue1Mob.Install(globalUeNodes.Get(0)); 
    
    // --- SETUP UE 3 ---
    MobilityHelper ue3Mob;
    Ptr<ListPositionAllocator> ue3Pos = CreateObject<ListPositionAllocator>();
    ue3Pos->Add(Vector(135, 220, 1.5)); // Vị trí khởi tạo nằm trong Bounds
    ue3Mob.SetPositionAllocator(ue3Pos);
    ue3Mob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                            "Bounds", RectangleValue(Rectangle(130, 180, 200, 240)), // Vùng đi quanh UAV 3
                            "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"),
                            "Mode", StringValue("Time"));
    ue3Mob.Install(globalUeNodes.Get(2)); 
    
    // UE2 (Moving UE)
    Ptr<ListPositionAllocator> p2 = CreateObject<ListPositionAllocator>(); p2->Add(Vector(75, 75, 1.5));
    mob.SetPositionAllocator(p2); mob.SetMobilityModel("ns3::WaypointMobilityModel"); mob.Install(globalUeNodes.Get(1));
    Ptr<WaypointMobilityModel> wp = globalUeNodes.Get(1)->GetObject<WaypointMobilityModel>();
    //wp->AddWaypoint(Waypoint(Seconds(0), Vector(80, 80, 1.5)));
    //wp->AddWaypoint(Waypoint(Seconds(30), Vector(190, 80, 1.5)));
    //wp->AddWaypoint(Waypoint(Seconds(60), Vector(150, 250, 1.5)));
    //wp->AddWaypoint(Waypoint(Seconds(99), Vector(220, 80, 1.5)));
    
    wp->AddWaypoint(Waypoint(Seconds(0.1), Vector(75, 75, 1.5)));   // Bắt đầu
    wp->AddWaypoint(Waypoint(Seconds(10.0), Vector(110, 76, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(20.0), Vector(145, 78, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(30.0), Vector(180, 79, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(40.0), Vector(220, 80, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(50.0), Vector(220, 100, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(60.0), Vector(200, 150, 1.5)));
    wp->AddWaypoint(Waypoint(Seconds(70.0), Vector(180, 180, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(80.0), Vector(150, 200, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(99.0), Vector(130, 220, 1.5)));
    
    // Building Environment
    BuildingsHelper::Install(uavEnbNodes); BuildingsHelper::Install(globalUeNodes);
    Ptr<GridBuildingAllocator> grid = CreateObject<GridBuildingAllocator>();
    grid->SetAttribute("GridWidth", UintegerValue(3)); grid->SetAttribute("LengthX", DoubleValue(20)); grid->Create(9);
    
    // Install LTE Devices
    NetDeviceContainer uavDevs = lteHelper->InstallEnbDevice(uavEnbNodes);
    NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(globalUeNodes);
    
    // Configure Spectrum & Power
    for(uint32_t i=0; i<uavDevs.GetN(); ++i) {
        uavDevs.Get(i)->GetObject<LteEnbNetDevice>()->SetAttribute("DlEarfcn", UintegerValue(100));
        uavDevs.Get(i)->GetObject<LteEnbNetDevice>()->SetAttribute("UlEarfcn", UintegerValue(18100));
        uavDevs.Get(i)->GetObject<LteEnbNetDevice>()->GetPhy()->SetAttribute("TxPower", DoubleValue(43.0));
    }
    for(uint32_t i=0; i<ueDevs.GetN(); ++i) ueDevs.Get(i)->GetObject<LteUeNetDevice>()->SetAttribute("DlEarfcn", UintegerValue(100));
    
    // IP Stack & Routing
    internet.Install(globalUeNodes);
    Ipv4InterfaceContainer ueIp = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
    for(uint32_t i=0; i<globalUeNodes.GetN(); ++i) ipv4RoutingHelper.GetStaticRouting(globalUeNodes.Get(i)->GetObject<Ipv4>())->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    
    // Attach UEs to initial UAVs
    lteHelper->Attach(ueDevs.Get(0), uavDevs.Get(0));
    lteHelper->Attach(ueDevs.Get(1), uavDevs.Get(0));
    lteHelper->Attach(ueDevs.Get(2), uavDevs.Get(2));
    lteHelper->AddX2Interface(uavEnbNodes); 
    
    // Apps (UDP Traffic)
    uint16_t port = 1234;
    ApplicationContainer apps;
    
    for(uint32_t i=0; i<globalUeNodes.GetN(); ++i) {
        // Cài Sink (Nhận tin) trên UE - Dùng UDP
        PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
        apps.Add(sink.Install(globalUeNodes.Get(i)));
        
        // 2. Cài Client (Gửi tin) trên Remote Host bắn về IP của UE
        UdpClientHelper client(ueIp.GetAddress(i), port);
        
        // Cấu hình: Gửi liên tục, mỗi gói 1472 bytes, cách nhau 4ms (~3 Mbps)
        client.SetAttribute("MaxPackets", UintegerValue(100000000));
        client.SetAttribute("Interval", TimeValue(MilliSeconds(4.0))); 
        client.SetAttribute("PacketSize", UintegerValue(1472));
        
        apps.Add(client.Install(remoteHost));
        
        // Khởi tạo lại các map theo dõi
        totalRxBytes[i+1] = 0; 
        lastThroughput[i+1] = 0.0;
        ueCurrentCellMap[i+1] = 0; 
        uePreviousMecCellMap[i+1] = 0;
    }
    // Start trễ 1 giây để mạng LTE kịp khởi động 
    apps.Start(Seconds(1.0));
    
    // MEC Simulation Loop
    for(double t=2.0; t<simTime; t+=2.0) Simulator::Schedule(Seconds(t), &GenerateMecTask, 2); 
    
    // Traces
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart", MakeCallback(&NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk", MakeCallback(&NotifyHandoverEndOkUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished", MakeCallback(&NotifyConnectionEstablished));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport", MakeCallback(&RecvMeasurementReportCallback));
    for(uint32_t i=0; i<ueDevs.GetN(); ++i) {
        ueDevs.Get(i)->GetObject<LteUeNetDevice>()->GetPhy()->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr", MakeBoundCallback(&ReportRsrp, ueDevs.Get(i)->GetObject<LteUeNetDevice>()->GetImsi()));
    }
    
    // Monitoring Schedules
    for(double t=1.0; t<simTime; t+=1.0) Simulator::Schedule(Seconds(t), &LogPositions, globalUeNodes, uavEnbNodes);
    for(double t=1.0; t<simTime; t+=0.1) {
        // apps.Get(0) -> Sink của UE 1
        Simulator::Schedule(Seconds(t), &CalculateThroughput, apps.Get(0)->GetObject<PacketSink>(), 1, 0.1);
        
        // apps.Get(2) -> Sink của UE 2
        Simulator::Schedule(Seconds(t), &CalculateThroughput, apps.Get(2)->GetObject<PacketSink>(), 2, 0.1);
        
        // apps.Get(4) -> Sink của UE 3
        Simulator::Schedule(Seconds(t), &CalculateThroughput, apps.Get(4)->GetObject<PacketSink>(), 3, 0.1);
    }
    
    // NetAnim
    pAnim = new AnimationInterface("scenario1_final.xml");
    pAnim->EnablePacketMetadata(false); 
    pAnim->SetMaxPktsPerTraceFile(999999999999ULL);
    
    pAnim->UpdateNodeDescription(uavEnbNodes.Get(0), "UAV-1 (Cell 1)"); pAnim->UpdateNodeColor(uavEnbNodes.Get(0), 255, 0, 0);
    pAnim->UpdateNodeDescription(uavEnbNodes.Get(1), "UAV-2 (Cell 2)"); pAnim->UpdateNodeColor(uavEnbNodes.Get(1), 0, 255, 0);
    pAnim->UpdateNodeDescription(uavEnbNodes.Get(2), "UAV-3 (Cell 3)"); pAnim->UpdateNodeColor(uavEnbNodes.Get(2), 128, 0, 128);
    
    // Remote Host (Server)
    pAnim->UpdateNodeDescription(remoteHost, "SERVER (Remote Host)");
    pAnim->UpdateNodeColor(remoteHost, 0, 0, 255); // Màu Xanh Dương Đậm
    pAnim->UpdateNodeSize(remoteHost, 2.0, 2.0); 

    // PGW
    pAnim->UpdateNodeDescription(pgw, "PGW (Gateway)");
    pAnim->UpdateNodeColor(pgw, 100, 100, 100); // Màu Xám
    
    // SGW 
    Ptr<Node> sgwNode = epcHelper->GetSgwNode();
    pAnim->UpdateNodeDescription(sgwNode, "SGW");
    pAnim->UpdateNodeColor(sgwNode, 150, 150, 150); // Xám nhạt
    
    pAnim->UpdateNodeDescription(mme, "MME (Control)");
    pAnim->UpdateNodeColor(mme, 200, 200, 200); // Màu xám nhạt hơn
    
    // Kích hoạt FlowMonitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
    std::cout << "Simulation Started..." << std::endl;
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    
    // Xuất kết quả FlowMonitor
    std::cout << "Processing FlowMonitor stats..." << std::endl;
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        // Tính Packet Loss Ratio
        double lossRate = 0.0;
        if (i->second.txPackets > 0) lossRate = (double)i->second.lostPackets / (double)i->second.txPackets * 100.0;
        
        flowStatsFile << i->first << "," << t.sourceAddress << "," << t.destinationAddress << ","
                      << i->second.txPackets << "," << i->second.rxPackets << ","
                      << i->second.lostPackets << "," << lossRate << ","
                      << i->second.delaySum.GetSeconds() / (i->second.rxPackets+1) * 1000 << ","
                      << i->second.jitterSum.GetSeconds() / (i->second.rxPackets > 1 ? i->second.rxPackets - 1 : 1) * 1000 << std::endl;
    }

    // Close Files & Clean up
    rsrpFile.close(); throughputFile.close(); mecOffloadFile.close(); cellIdFile.close(); 
    uavPositionFile.close(); positionFile.close(); handoverFile.close(); handoverQualityFile.close(); sinrFile.close(); hoTraceFile.close();
    flowStatsFile.close();
    
    std::cout << "\n=== FINAL REPORT ===" << std::endl;
    std::cout << "HO Attempts: " << handoverStartCount << " | Success: " << handoverCount << std::endl;
    std::cout << "Stats Generated in: scenario1_final_flow_stats.csv, mec_offload.csv, etc." << std::endl;
    
    delete pAnim; 
    Simulator::Destroy();
    return 0;
}
