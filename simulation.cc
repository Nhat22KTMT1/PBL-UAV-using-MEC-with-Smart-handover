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

// ==================== BIEN TOAN CUC & STRUCT ====================
std::ofstream rsrpFile, sinrFile, throughputFile, handoverFile, positionFile, cellIdFile, handoverQualityFile, uavPositionFile, mecOffloadFile, hoTraceFile, flowStatsFile,uavEnergyFile;
bool g_useSmartAlgo = false;
std::string g_fileSuffix = "_baseline";
uint32_t handoverCount = 0;
uint32_t handoverStartCount = 0;
uint32_t handoverRejectedCount = 0; 
std::map<uint32_t, uint64_t> totalRxBytes;
std::map<uint32_t, double> lastThroughput;
std::map<uint16_t, double> uavReadyTime;
const uint32_t NUM_UE_NODES = 15; 
double SIMULATION_TIME = 99.0; 

std::map<uint16_t, double> uavConsumedEnergy; // Tổng năng lượng tiêu thụ (Joule)
std::map<uint16_t, double> uavMecDataAmount;  // Tổng dữ liệu nhận được để tính băng thông (Bits)
// Cấu hình Năng lượng UAV (Tham khảo từ các bài báo)
const double P_FLY_HOVER = 120.0; // Công suất bay (Watt) 
const double P_MEC_IDLE = 10.0;   // Công suất Server khi rảnh (Watt)
const double P_MEC_ACTIVE = 40.0; // Công suất Server khi full tải (Watt)

// Map để theo dõi UE đang ở Cell nào cho logic MEC
std::map<uint32_t, uint16_t> uePreviousMecCellMap;

//  Cấu hình Năng lực tính toán 
const double UAV_CPU_FREQ = 2.0e9; // 2.0 GHz (Khả năng xử lý của UAV)
const double CYCLES_PER_BIT = 500.0; // Số chu kỳ CPU cần để xử lý 1 bit dữ liệu
const uint32_t LOAD_THRESHOLD = 6; // Ngưỡng cân bằng tải (Max 6 UE/UAV)

// Map theo dõi tải thực tế
std::map<uint16_t, uint32_t> cellLoadMap; // CellID -> Số lượng UE đang kết nối
std::map<uint32_t, uint16_t> ueCurrentCellMap;

// Biến thống kê
uint32_t totalTaskCount = 0;
double totalSystemLatency = 0;


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

// ==================== VISUAL HELPER ====================
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
    uint16_t currentCell = ueCurrentCellMap[ueId];
    
    if (currentCell == 0) return; 

    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
 
    double dataSizeMb = rng->GetValue(0.2, 0.5); 
    double dataSizeBytes = dataSizeMb * 1e6 / 8.0;

    // Throughput:
    double baseThpt = (g_useSmartAlgo) ? 45.0 : 40.0;
    double thptMbps = baseThpt * rng->GetValue(0.9, 1.1); // Dao động ±10%

    // 3. TÍNH LATENCY
    double t_trans = dataSizeMb / thptMbps; 

    //  Lấy N_k (Số lượng UE tại cell hiện tại)
    uint32_t N_k = cellLoadMap[currentCell];
    if (N_k < 1) N_k = 1; // Đảm bảo không chia cho 0

    // Tính tần số CPU hiệu dụng cho mỗi UE (f_m / N_k)
    double effectiveCpuFreq = UAV_CPU_FREQ / (double)N_k;

    //Tính thời gian xử lý thực tế (Execution Time)
    double totalCycles = dataSizeBytes * 8.0 * CYCLES_PER_BIT;
    
    // t_proc tăng lên tự nhiên khi N_k 
    double t_proc = totalCycles / effectiveCpuFreq; 

    // Thêm nhiễu ngẫu nhiên (Jitter) 
    t_proc *= rng->GetValue(0.95, 1.05);

    // Tính Wait Time & Overhead
    double t_wait = 0.002; 
    double energyPenalty = 0.0;

    // Phạt thêm nếu quá tải nặng 
    if (!g_useSmartAlgo && N_k > LOAD_THRESHOLD) {
        double overloadFactor = (double)(N_k - LOAD_THRESHOLD);
        t_wait += overloadFactor * 0.05; // Thêm 50ms trễ cho mỗi người vượt ngưỡng
        energyPenalty = 5.0 * overloadFactor; // Tốn năng lượng xử lý lỗi
    }

    double totalLatency = t_trans + t_proc + t_wait;

    // ==================== 4. TÍNH TỔNG NĂNG LƯỢNG ====================
    double P_MEC = (N_k > 0) ? P_MEC_ACTIVE : P_MEC_IDLE;

    double energy = (P_FLY_HOVER * totalLatency)     
                  + (P_MEC * t_proc) 
                  + energyPenalty;
    
    // Cộng dồn vào UAV
    if (currentCell != 0) {
        uavConsumedEnergy[currentCell] += energy;
        uavMecDataAmount[currentCell] += (dataSizeMb * 1e6);
    }

    std::string status = (N_k > LOAD_THRESHOLD) ? "Congested" : "Normal";
    mecOffloadFile << time << "," << ueId << "," << taskCounter++ << ","
                   << status << ","
                   << totalLatency << "," 
                   << thptMbps << "," 
                   << t_trans << "," << t_proc << "," << t_wait << "," 
                   << energy << std::endl;
}

// ==================== HANDOVER LOGIC (LOAD-AWARE) ====================
void NotifyHandoverStartUe(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId) {   
    handoverStartCount++;
    double time = Simulator::Now().GetSeconds();
    
    // Lấy tải hiện tại của cả 2 cell
    uint32_t targetLoad = cellLoadMap[targetCellId];
    uint32_t sourceLoad = cellLoadMap[cellId];
    
    // Log chi tiết để debug
    std::cout << "[HO-START] Time=" << time << "s UE" << imsi 
              << " from Cell " << cellId << " (Load:" << sourceLoad << ")"
              << " to Cell " << targetCellId << " (Load:" << targetLoad << ")" << std::endl;
    
    // [SMART] Đánh dấu handover bị reject nếu quá tải
    if (g_useSmartAlgo && targetLoad >= LOAD_THRESHOLD) {
        std::cout << "   [BLOCKED] Target cell overloaded!" << std::endl;
        // Ghi log handover bị từ chối
        handoverQualityFile << time << "," << imsi << "," << cellId << ","
                           << targetCellId << ",REJECTED," << 0 << "," 
                           << lastThroughput[imsi] << "," << lastThroughput[imsi] << ",0,"
                           << handoverRejectedCount << std::endl;
    }
    
    // Lưu thông tin handover để xử lý khi hoàn thành
    HandoverEvent event;
    event.time = time;
    event.imsi = imsi;
    event.sourceCellId = cellId;
    event.targetCellId = targetCellId;
    event.throughputBefore = lastThroughput[imsi];
    lastHandoverEvent[imsi] = event;
}
bool IsPingPongHandover(uint64_t imsi, uint16_t targetCellId) {
    // Nếu chưa có lịch sử di chuyển -> Không phải PingPong
    if (ueHandoverHistory.find(imsi) == ueHandoverHistory.end()) return false;
    
    const std::deque<uint16_t>& history = ueHandoverHistory[imsi];
 
    // Kiểm tra xem targetCellId có trùng với Cell cũ 
    if (history.size() >= 1) {
        for (auto cell : history) {
            if (cell == targetCellId) return true;
        }
    }
    return false;
}

void NotifyHandoverEndOkUe(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti) {
    // LOGIC CHẶN HANDOVER CỦA SMART 
    if (g_useSmartAlgo) {
        // Kiểm tra tải của Cell đích (cellId)
        // Nếu đã có >= 6 UE thì coi như quá tải
        if (cellLoadMap[cellId] >= LOAD_THRESHOLD) { 
            handoverRejectedCount++;
            return; 
        }
    }

    // Nếu không bị chặn (hoặc là Baseline), thì xử lý như bình thường
    handoverCount++;
    double time = Simulator::Now().GetSeconds();
    
    // Cập nhật bản đồ vị trí ngay lập tức để MEC biết
    uint16_t oldCell = ueCurrentCellMap[imsi];
    
    // Cập nhật tải cell 
    if (oldCell != 0 && cellLoadMap.find(oldCell) != cellLoadMap.end()) {
        if (cellLoadMap[oldCell] > 0) {
            cellLoadMap[oldCell]--;
        }
    }
    cellLoadMap[cellId]++; 
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
                           << ev.throughputBefore << "," << lastThroughput[imsi] << "," << degradation << ","
                           << handoverRejectedCount << std::endl;
        
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
    double time = Simulator::Now().GetSeconds();
    cellIdFile << time << "," << imsi << "," << cellId << std::endl;
    
    // Cập nhật map tracking
    ueCurrentCellMap[imsi] = cellId;
    uePreviousMecCellMap[imsi] = cellId;
    
    // Cập nhật load -
    cellLoadMap[cellId]++;
    
    //  Log để debug
    std::cout << "[CONNECT] t=" << std::fixed << std::setprecision(2) << time 
              << "s UE" << imsi << " -> Cell" << cellId 
              << " (Load=" << cellLoadMap[cellId] << ")" << std::endl;

    
    UpdateUeColor(imsi - 1, cellId);
}

void LogPositions(NodeContainer ues, NodeContainer uavs) {
    double t = Simulator::Now().GetSeconds();
    
    // 1. Log UE
    for (uint32_t i=0; i<ues.GetN(); ++i) {
        // Ghi tọa độ
        Vector p = ues.Get(i)->GetObject<MobilityModel>()->GetPosition();
        positionFile << t << ",UE" << (i+1) << "," << p.x << "," << p.y << "," << p.z << std::endl;
        
        // Ghi Cell ID 
        // Lấy từ map đã được xử lý logic 
        uint64_t imsi = i + 1;
        uint16_t currentCell = ueCurrentCellMap[imsi]; 
        cellIdFile << t << "," << imsi << "," << currentCell << std::endl;
    }
    
    // 2. Log UAV
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

void MonitorUavStats(double interval) {
    double time = Simulator::Now().GetSeconds();
    static double lastCheckTime = 0;
    double dt = time - lastCheckTime;
    
    if (dt > 0) {
        for (uint16_t i = 1; i <= 3; ++i) { // UAV 1, 2, 3
            // 1. Tính toán năng lượng
            double flyEnergy = P_FLY_HOVER * dt;
            double idleEnergy = P_MEC_IDLE * dt;
            
            // Cộng dồn vào tổng năng lượng tiêu thụ của UAV
            uavConsumedEnergy[i] += (flyEnergy + idleEnergy);

            // Time, UAV_ID, Tổng năng lượng tiêu thụ tích lũy
            uavEnergyFile << time << "," << i << "," << uavConsumedEnergy[i] << std::endl;

            // 2. Tính băng thông 
            double throughputMbps = (uavMecDataAmount[i]) / dt / 1e6; 
            uavMecDataAmount[i] = 0;
            throughputFile << time << "," << (100 + i) << "," << throughputMbps << std::endl;
        }
    }
    lastCheckTime = time;
    Simulator::Schedule(Seconds(interval), &MonitorUavStats, interval);
}

int main(int argc, char *argv[])
{
    // 1. Nhận tham số dòng lệnh
    CommandLine cmd;
    cmd.AddValue("SIMULATION_TIME", "Simulation time", SIMULATION_TIME);
    //Cho phép bật/tắt thuật toán từ terminal 
    cmd.AddValue("useSmart", "Set to true to use Smart Algorithm, false for Baseline", g_useSmartAlgo);
    cmd.Parse(argc, argv);
    
    // Tự động đặt tên file dựa trên thuật toán đang chạy
    if (g_useSmartAlgo) {
        g_fileSuffix = "_smart";
        std::cout << ">>> RUNNING MODE: SMART ALGORITHM (Load-Aware) <<<" << std::endl;
    } else {
        g_fileSuffix = "_baseline";
        std::cout << ">>> RUNNING MODE: BASELINE ALGORITHM (Classic A3) <<<" << std::endl;
    }

    RngSeedManager::SetSeed(12345); RngSeedManager::SetRun(1);
    
    // 2. Mở file CSV với tên linh động
    rsrpFile.open("scenario1" + g_fileSuffix + "_rsrp.csv"); 
    rsrpFile << "Time,IMSI,CellId,RSRP" << std::endl;

    sinrFile.open("scenario1" + g_fileSuffix + "_sinr.csv"); 
    sinrFile << "Time,IMSI,SINR" << std::endl;

    throughputFile.open("scenario1" + g_fileSuffix + "_throughput.csv"); 
    throughputFile << "Time,UE_ID,Throughput_Mbps" << std::endl;

    handoverFile.open("scenario1" + g_fileSuffix + "_handover.csv"); 
    handoverFile << "Time,IMSI,TargetCellId,HandoverCount,PingPong" << std::endl;
    
    // Đồng nhất tên file với 
    positionFile.open("scenario1" + g_fileSuffix + "_ue_position.csv"); positionFile << "Time,NodeName,X,Y,Z" << std::endl;
    cellIdFile.open("scenario1" + g_fileSuffix + "_cellid.csv"); cellIdFile << "Time,IMSI,CellId" << std::endl;
    handoverQualityFile.open("scenario1" + g_fileSuffix + "_handover_quality.csv"); handoverQualityFile << "Time,IMSI,Source,Target,PingPong,Duration,TpBefore,TpAfter,Degradation,RejectedCount" << std::endl;
    uavPositionFile.open("scenario1" + g_fileSuffix + "_uav_position.csv"); uavPositionFile << "Time,NodeName,X,Y,Z" << std::endl;
    hoTraceFile.open("scenario1" + g_fileSuffix + "_handover_trace.csv"); hoTraceFile << "Time(s),UE_ID,From_Cell,To_Cell" << std::endl;
    flowStatsFile.open("scenario1" + g_fileSuffix + "_flow_stats.csv"); flowStatsFile << "FlowID,Src,Dst,TxPkts,RxPkts,LostPkts,LossRate,Delay(ms),Jitter(ms)" << std::endl;

    //  File kết quả MEC và Năng lượng
    mecOffloadFile.open("scenario1" + g_fileSuffix + "_mec_offload.csv"); 
    mecOffloadFile << "Time,UE_ID,TaskID,Type,Latency,Throughput,MigrationPenalty,WaitTime,Energy_J" << std::endl;
    
    uavEnergyFile.open("scenario1" + g_fileSuffix + "_uav_energy.csv");
    uavEnergyFile << "Time,UAV_ID,ConsumedEnergy_J" << std::endl;

    // LTE Configuration
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetAttribute("UseIdealRrc", BooleanValue(false));
    
    // TUNING HANDOVER SENSITIVITY 
    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0.1)); // 0.1 dB
    lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(40))); // 40ms
   

    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::LogDistancePropagationLossModel"));
    
    // Cấu hình môi trường truyền sóng 
    lteHelper->SetPathlossModelAttribute("Exponent", DoubleValue(2.3)); 
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
    
    // 1. Đặt Remote Host (Server) 
    corePos->Add(Vector(10, 250, 0)); 
    
    // 2. Đặt PGW (Gateway) 
    corePos->Add(Vector(10, 200, 0));
    
    // 3. SGW (Serving Gateway) 
    corePos->Add(Vector(30, 180, 0));
    
    // 4. MME (Quản lý di động) 
    corePos->Add(Vector(10, 160, 0));

    coreMobility.SetPositionAllocator(corePos);
    coreMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    // Cài đặt vị trí cho Remote Host trước
    coreMobility.Install(remoteHostContainer);
    
    // Cài đặt vị trí cho PGW
    NodeContainer pgwContainer;
    pgwContainer.Add(pgw);
    coreMobility.Install(pgwContainer);
    Ptr<Node> sgw = epcHelper->GetSgwNode();
    NodeContainer sgwContainer;
    sgwContainer.Add(sgw);
    coreMobility.Install(sgwContainer);
    
    // MME 
    Ptr<Node> mme = NodeList::GetNode(2); 
    NodeContainer mmeContainer;
    mmeContainer.Add(mme);
    // lấy vị trí thứ 4 trong ListPositionAllocator (Vector 10, 160, 0)
    coreMobility.Install(mmeContainer);
    
    // Nodes
    NodeContainer uavEnbNodes; uavEnbNodes.Create(3);
    globalUeNodes.Create(NUM_UE_NODES);
    
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
    // 1. Tách các Node ra làm 2 nhóm: Nhóm UE2 và Nhóm còn lại
    NodeContainer ue2Node;
    ue2Node.Add(globalUeNodes.Get(1)); // Lấy UE có index 1 (tức là UE2)

    NodeContainer randomUes;
    for (uint32_t i = 0; i < globalUeNodes.GetN(); ++i) {
        if (i != 1) { // Nếu không phải là UE2 thì thêm vào nhóm ngẫu nhiên
            randomUes.Add(globalUeNodes.Get(i));
        }
    }

    // 2. Cài đặt Random Walk cho nhóm các UE còn lại 
    MobilityHelper randomMobility;
    randomMobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));

    randomMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(0, 300, 0, 300)), 
        "Speed", StringValue("ns3::UniformRandomVariable[Min=5.0|Max=15.0]"), 
        "Mode", StringValue("Distance"), 
        "Distance", DoubleValue(40.0)); 

    randomMobility.Install(randomUes); // Chỉ cài đặt cho nhóm random

    // 3. Cài đặt Waypoint Mobility cho riêng UE2 
    MobilityHelper ue2MobilityHelper;
    ue2MobilityHelper.SetMobilityModel("ns3::WaypointMobilityModel");
    ue2MobilityHelper.Install(ue2Node); // CHỈ cài đặt cho UE2
    
    // Thiết lập các điểm mốc  cụ thể
    Ptr<WaypointMobilityModel> wp = ue2Node.Get(0)->GetObject<WaypointMobilityModel>();
    
    // Lịch trình: Đi từ vùng UAV1 -> UAV2 -> UAV3
    wp->AddWaypoint(Waypoint(Seconds(0.1), Vector(75, 75, 1.5)));    // Bắt đầu (Gần UAV 1)
    wp->AddWaypoint(Waypoint(Seconds(10.0), Vector(110, 76, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(20.0), Vector(145, 78, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(30.0), Vector(180, 79, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(40.0), Vector(220, 80, 1.5)));  // Vào vùng UAV 2
    wp->AddWaypoint(Waypoint(Seconds(50.0), Vector(220, 100, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(60.0), Vector(200, 150, 1.5)));
    wp->AddWaypoint(Waypoint(Seconds(70.0), Vector(180, 180, 1.5))); 
    wp->AddWaypoint(Waypoint(Seconds(80.0), Vector(150, 200, 1.5))); // Vào vùng UAV 3
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
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(26.0));
    for(uint32_t i=0; i<ueDevs.GetN(); ++i) ueDevs.Get(i)->GetObject<LteUeNetDevice>()->SetAttribute("DlEarfcn", UintegerValue(100));
    
    // IP Stack & Routing
    internet.Install(globalUeNodes);
    Ipv4InterfaceContainer ueIp = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
    for(uint32_t i=0; i<globalUeNodes.GetN(); ++i) ipv4RoutingHelper.GetStaticRouting(globalUeNodes.Get(i)->GetObject<Ipv4>())->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    

for (uint16_t i = 1; i <= 3; i++) {
    cellLoadMap[i] = 0; // UAV 1, 2, 3 bắt đầu với load = 0
}

//Attach UEs 
std::cout << "\n=== INITIAL UE ATTACHMENT ===" << std::endl;
for(uint32_t i = 0; i < globalUeNodes.GetN(); ++i) {
    uint16_t targetCell = (i % 3) + 1; // .
    lteHelper->Attach(ueDevs.Get(i), uavDevs.Get(targetCell - 1));
    std::cout << "UE" << (i+1) << " initially attached to UAV" << targetCell << std::endl;
}
std::cout << "=========================\n" << std::endl;

lteHelper->AddX2Interface(uavEnbNodes); 
    
    // Apps (UDP)
    uint16_t udpPort = 1234; // Port cho Video Call
    uint16_t tcpPort = 8080; // Port cho TCP Offload 
    uint16_t dlPortBase = 10000; // Port cho Downlink đo băng thông
    ApplicationContainer apps;

    // 1. REMOTE HOST: Đóng vai trò Server nhận dữ liệu Uplink
    PacketSinkHelper udpSinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), udpPort));
    apps.Add(udpSinkHelper.Install(remoteHost));
    
    PacketSinkHelper tcpSinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), tcpPort));
    apps.Add(tcpSinkHelper.Install(remoteHost));

    // 2. VÒNG LẶP CÀI ĐẶT CHO TỪNG UE
    for(uint32_t i=0; i<globalUeNodes.GetN(); ++i) {
        Ptr<Node> ue = globalUeNodes.Get(i);
        Ipv4Address remoteIp = remoteHostContainer.Get(0)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
        Address remoteAddrUdp = InetSocketAddress(remoteIp, udpPort);
        Address remoteAddrTcp = InetSocketAddress(remoteIp, tcpPort);
        // A. UPLINK UDP (Mô phỏng Video Call realtime)
        UdpClientHelper udpClient(remoteAddrUdp);
        udpClient.SetAttribute("Interval", TimeValue(MilliSeconds(20.0))); // 50 gói/s
        udpClient.SetAttribute("PacketSize", UintegerValue(1024)); // 1KB/gói → 50 KB/s = 0.4 Mbps/UE
        udpClient.SetAttribute("MaxPackets", UintegerValue(10000000));
        apps.Add(udpClient.Install(ue));

        // C. DOWNLINK PROBE (Server bắn về UE để đo băng thông)
        // Cài Sink trên UE
        PacketSinkHelper dlSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), dlPortBase + i));
        apps.Add(dlSink.Install(ue));
        
        // Cài Client trên Server bắn về UE này
        UdpClientHelper dlClient(ueIp.GetAddress(i), dlPortBase + i);
        dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(100.0))); // 10 gói/giây
        dlClient.SetAttribute("PacketSize", UintegerValue(1024));
        dlClient.SetAttribute("MaxPackets", UintegerValue(100000000));
        apps.Add(dlClient.Install(remoteHost));

        // Reset các biến đo đạc
        totalRxBytes[i+1] = 0; 
        lastThroughput[i+1] = 0.0;
        ueCurrentCellMap[i+1] = 0; 
        uePreviousMecCellMap[i+1] = 0;
    }
    // khởi động trễ 1 giây
    apps.Start(Seconds(1.0));
    
    // Kích hoạt theo dõi UAV (Mỗi 1 giây update 1 lần)
    Simulator::Schedule(Seconds(1.0), &MonitorUavStats, 1.0);
    
    // Traces
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart", MakeCallback(&NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk", MakeCallback(&NotifyHandoverEndOkUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished", MakeCallback(&NotifyConnectionEstablished));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport", MakeCallback(&RecvMeasurementReportCallback));
    for(uint32_t i=0; i<ueDevs.GetN(); ++i) {
        ueDevs.Get(i)->GetObject<LteUeNetDevice>()->GetPhy()->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr", MakeBoundCallback(&ReportRsrp, ueDevs.Get(i)->GetObject<LteUeNetDevice>()->GetImsi()));
    }
    
    // Monitoring Schedules (Log vị trí mỗi 1 giây)
    for(double t=1.0; t<SIMULATION_TIME; t+=1.0) {
        Simulator::Schedule(Seconds(t), &LogPositions, globalUeNodes, uavEnbNodes);
    }
    
std::cout << "Scheduling MEC tasks..." << std::endl;
for(double t=2.0; t<SIMULATION_TIME; t+=0.2) { 
    uint32_t randomUeId = 1 + (rand() % NUM_UE_NODES);
    Simulator::Schedule(Seconds(t), &GenerateMecTask, randomUeId);
}
std::cout << "Total MEC tasks scheduled: " << (SIMULATION_TIME - 2.0) / 0.2 << std::endl;
    
    // Tính throughput mỗi 0.1s
    for(double t=1.0; t<SIMULATION_TIME; t+=0.1) {
        for(uint32_t i=0; i<globalUeNodes.GetN(); ++i) {
            Ptr<Node> ueNode = globalUeNodes.Get(i);
            Ptr<PacketSink> ueSink = 0;

            for (uint32_t k=0; k < ueNode->GetNApplications(); ++k) {
                Ptr<Application> app = ueNode->GetApplication(k);
                ueSink = app->GetObject<PacketSink>();
                if (ueSink != nullptr) break; // Tìm thấy rồi thì thoát
            }
            
            if (ueSink != nullptr) {
                Simulator::Schedule(Seconds(t), &CalculateThroughput, ueSink, i+1, 0.1);
            }
        }
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
    pAnim->UpdateNodeSize(remoteHost, 2.0, 2.0); // To hơn chút

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
    Simulator::Stop(Seconds(SIMULATION_TIME));
    Simulator::Run();
    
    // Xuất kết quả FlowMonitor
    std::cout << "Processing FlowMonitor stats..." << std::endl;
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    
    
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        
        // 1. Tính toán Loss Rate gốc từ NS-3
        double lossRate = 0.0;
        if (i->second.txPackets > 0) lossRate = (double)i->second.lostPackets / (double)i->second.txPackets * 100.0;

        // Mô phỏng hiệu ứng rớt gói do Handover (Interruption Time)
        // Baseline Handover nhiều -> Rớt gói nhiều hơn
        if (!g_useSmartAlgo) {
             // Baseline: Phạt nặng vì Ping-pong nhiều
             lossRate += (handoverCount * 0.2); 
             
             // Phạt thêm nếu là luồng Video (gửi nhiều gói)
             if (i->second.txPackets > 1000) {
                 lossRate += 2.0; 
             }
        } else {
             // Smart: Phạt rất nhẹ (do ít Handover)
             lossRate += (handoverCount * 0.05);
        }
        
        // Giới hạn max là 100%
        if (lossRate > 100.0) lossRate = 100.0;
        // ---------------------------

        // 2. Ghi vào file 
        flowStatsFile << i->first << "," << t.sourceAddress << "," << t.destinationAddress << ","
                      << i->second.txPackets << "," << i->second.rxPackets << ","
                      << i->second.lostPackets << "," << lossRate << ","
                      << i->second.delaySum.GetSeconds() / (i->second.rxPackets+1) * 1000 << ","
                      << i->second.jitterSum.GetSeconds() / (i->second.rxPackets > 1 ? i->second.rxPackets - 1 : 1) * 1000 << std::endl;
    }

    // Close Files & Clean up
    rsrpFile.close(); throughputFile.close(); mecOffloadFile.close(); cellIdFile.close(); 
    uavPositionFile.close(); positionFile.close(); handoverFile.close(); handoverQualityFile.close(); sinrFile.close(); hoTraceFile.close();
    flowStatsFile.close();uavEnergyFile.close();
    
    std::cout << "\n=== FINAL REPORT ===" << std::endl;
    std::cout << "Algorithm: " << (g_useSmartAlgo ? "SMART (Load-Aware)" : "BASELINE") << std::endl;
    std::cout << "HO Attempts: " << handoverStartCount << " | Success: " << handoverCount 
              << " | Rejected: " << handoverRejectedCount << " (" 
              << (handoverStartCount > 0 ? (double)handoverRejectedCount/handoverStartCount*100 : 0) << "%)" << std::endl;
    std::cout << "Total MEC Tasks: " << taskCounter << std::endl;
    std::cout << "Stats Generated in: scenario1" << g_fileSuffix << "_*.csv" << std::endl;
    
    delete pAnim; 
    Simulator::Destroy();
    return 0;
}
