import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# === CẤU HÌNH GIAO DIỆN ===
try:
    plt.style.use('seaborn-v0_8-whitegrid')
except OSError:
    sns.set_style("whitegrid")

plt.rcParams['figure.figsize'] = (24, 12)
plt.rcParams['lines.linewidth'] = 2.0
plt.rcParams['font.size'] = 10

def safe_read_csv(filename):
    try:
        return pd.read_csv(filename)
    except FileNotFoundError:
        print(f"⚠️ Lỗi: Không tìm thấy file {filename}")
        return None

# === LOAD TẤT CẢ DỮ LIỆU ===
df_thpt = safe_read_csv('scenario1_final_throughput.csv')
df_flow = safe_read_csv('scenario1_final_flow_stats.csv')
df_rsrp = safe_read_csv('scenario1_final_rsrp.csv')
df_mec = safe_read_csv('scenario1_final_mec_offload.csv')
df_ho = safe_read_csv('scenario1_final_handover_quality.csv')
df_sinr = safe_read_csv('scenario1_final_sinr.csv')

# === TẠO KHUNG HÌNH 2 HÀNG x 4 CỘT ===
fig, axes = plt.subplots(2, 4)
plt.subplots_adjust(hspace=0.35, wspace=0.3)

# ========================================================
# HÀNG 1: 4 BIỂU ĐỒ ĐẦU
# ========================================================

# --- (1,1) UE 1 THROUGHPUT (STATIC) ---
ax1 = axes[0, 0]
if df_thpt is not None:
    ue1 = df_thpt[df_thpt['UE_ID'] == 1]
    if not ue1.empty:
        ax1.plot(ue1['Time'].values, ue1['Throughput_Mbps'].values, color='#1f77b4', label='Throughput')
        ax1.axhline(y=3.0, color='red', linestyle=':', alpha=0.5, label='Expected (3Mbps)')
        ax1.set_title('(A) UE 1: Throughput', fontweight='bold', fontsize=11)
        ax1.set_ylabel('Mbps')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylim(0, 5)
        ax1.legend(fontsize=8)

# --- (1,2) UE 3 THROUGHPUT (STATIC) ---
ax2 = axes[0, 1]
if df_thpt is not None:
    ue3 = df_thpt[df_thpt['UE_ID'] == 3]
    if not ue3.empty:
        ax2.plot(ue3['Time'].values, ue3['Throughput_Mbps'].values, color='#ff7f0e', label='Throughput')
        ax2.axhline(y=3.0, color='red', linestyle=':', alpha=0.5, label='Expected (3Mbps)')
        ax2.set_title('(B) UE 3: Throughput', fontweight='bold', fontsize=11)
        ax2.set_ylabel('Mbps')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylim(0, 5)
        ax2.legend(fontsize=8)

# --- (1,3) SO SÁNH TỔNG HỢP (Mobile vs Static) ---
ax3 = axes[0, 2]
if df_thpt is not None:
    ue1 = df_thpt[df_thpt['UE_ID'] == 1]
    ue3 = df_thpt[df_thpt['UE_ID'] == 3]
    ue2 = df_thpt[df_thpt['UE_ID'] == 2]
    
    if not ue1.empty:
        ax3.plot(ue1['Time'].values, ue1['Throughput_Mbps'].values, label='UE 1', color='#1f77b4', alpha=0.4)
    if not ue3.empty:
        ax3.plot(ue3['Time'].values, ue3['Throughput_Mbps'].values, label='UE 3', color='#ff7f0e', alpha=0.4)
    if not ue2.empty:
        ax3.plot(ue2['Time'].values, ue2['Throughput_Mbps'].values, label='UE 2 (Mobile)', color='red', linewidth=2.5)

    ax3.set_title('(C) Throughput comparison between 3 UEs', fontweight='bold', fontsize=11, color='darkred')
    ax3.set_ylabel('Throughput (Mbps)')
    ax3.set_xlabel('Time (s)')
    ax3.legend(fontsize=8)
    ax3.grid(True, linestyle='--', alpha=0.7)

# --- (1,4) JITTER COMPARISON (BAR CHART) ---
ax4 = axes[0, 3]
if df_flow is not None:
    col_tx = 'TxPkts' if 'TxPkts' in df_flow.columns else 'TxPackets'
    df_clean = df_flow[df_flow[col_tx].values > 100].copy()
    df_clean = df_clean.sort_values('FlowID')
    
    labels = []
    colors = []
    for fid in df_clean['FlowID']:
        if fid == 7:
            labels.append("UE 2")
            colors.append('red')
        elif fid == 1:
            labels.append("UE 1")
            colors.append('skyblue')
        elif fid == 3:
            labels.append("UE 3")
            colors.append('skyblue')
        else:
            labels.append(f"UE {fid}")
            colors.append('skyblue')

    if not df_clean.empty:
        bars = ax4.bar(labels, df_clean['Jitter(ms)'].values, color=colors, alpha=0.9, edgecolor='black')
        ax4.set_title('(D) Jitter Comparison', fontweight='bold', fontsize=11)
        ax4.set_ylabel('Jitter (ms)')
        ax4.set_ylim(0, df_clean['Jitter(ms)'].max() * 1.25)
        
        for bar in bars:
            height = bar.get_height()
            ax4.text(bar.get_x() + bar.get_width()/2., height + 0.02,
                     f'{height:.2f}', ha='center', va='bottom', fontweight='bold', fontsize=8)

# ========================================================
# HÀNG 2: 4 BIỂU ĐỒ TIẾP THEO
# ========================================================

# --- (2,1) RSRP (Signal Strength) ---
ax5 = axes[1, 0]
if df_rsrp is not None and not df_rsrp.empty:
    target_ue = 2
    ue_data = df_rsrp[df_rsrp['IMSI'] == target_ue]
    if not ue_data.empty:
        pivot_data = ue_data.pivot_table(index='Time', columns='CellId', values='RSRP')
        pivot_data.plot(ax=ax5, marker='.', markersize=2, alpha=0.8, legend=True)
        
        if df_ho is not None and not df_ho.empty:
            ho_events = df_ho[df_ho['IMSI'] == target_ue]
            for _, row in ho_events.iterrows():
                ax5.axvline(x=row['Time'], color='red', linestyle='--', linewidth=1.2, alpha=0.8)

        ax5.set_title(f'(E) RSRP & Handover (UE {target_ue})', fontweight='bold', fontsize=11)
        ax5.set_ylabel('RSRP (dBm)')
        ax5.set_xlabel('Time (s)')
        ax5.legend(title='Cell', fontsize=7, loc='lower right')

# --- (2,2) THROUGHPUT WITH HANDOVER IMPACT ---
ax6 = axes[1, 1]
if df_thpt is not None and not df_thpt.empty:
    target_ue = 2
    thpt_data = df_thpt[df_thpt['UE_ID'] == target_ue]
    if not thpt_data.empty:
        ax6.plot(thpt_data['Time'].values, thpt_data['Throughput_Mbps'].values, color='#2ca02c', label='Throughput')
        if df_ho is not None and not df_ho.empty:
            ho_events = df_ho[df_ho['IMSI'] == target_ue]
            for _, row in ho_events.iterrows():
                ax6.axvspan(row['Time']-2, row['Time']+2, color='red', alpha=0.15)
        
        ax6.set_title('(F) Throughput Stability (UE 2)', fontweight='bold', fontsize=11)
        ax6.set_ylabel('Mbps')
        ax6.set_xlabel('Time (s)')
        ax6.legend(fontsize=8)

# --- (2,3) MEC LATENCY ---
ax7 = axes[1, 2]
if df_mec is not None and not df_mec.empty:
    target_ue = 2
    mec_data = df_mec[df_mec['UE_ID'] == target_ue].copy()
    if not mec_data.empty:
        mec_data['Latency_ms'] = mec_data['Latency'] * 1000
        ax7.plot(mec_data['Time'].values, mec_data['Latency_ms'].values, 
                 marker='o', markersize=4, color='#1f77b4', label='Latency')
        
        ax7.set_title('(G) MEC Service Latency (UE 2)', fontweight='bold', fontsize=11)
        ax7.set_ylabel('Latency (ms)')
        ax7.set_xlabel('Time (s)')
        ax7.legend(fontsize=8)

# --- (2,4) SINR VARIATION ---
ax8 = axes[1, 3]
if df_sinr is not None and not df_sinr.empty:
    target_ue = 2
    ue_sinr = df_sinr[df_sinr['IMSI'] == target_ue]
    if not ue_sinr.empty:
        ue_sinr_clean = ue_sinr[ue_sinr['SINR'] < 60]
        ax8.plot(ue_sinr_clean['Time'].values, ue_sinr_clean['SINR'].values, 
                 color='purple', alpha=0.8, label='SINR')
        
        if df_ho is not None and not df_ho.empty:
            ho_events = df_ho[df_ho['IMSI'] == target_ue]
            for _, row in ho_events.iterrows():
                ax8.axvline(x=row['Time'], color='red', linestyle='--', linewidth=1.2)

        ax8.set_title(f'(H) SINR with Nakagami Fading (UE {target_ue})', fontweight='bold', fontsize=11)
        ax8.set_ylabel('SINR (dB)')
        ax8.set_xlabel('Time (s)')
        ax8.set_ylim(-10, 60)
        ax8.legend(fontsize=8, loc='lower right')

# === LƯU FILE ẢNH ===
plt.tight_layout()
plt.savefig('Full_UAV_MEC_Report_2x4.png', dpi=300, bbox_inches='tight')
print("Đã xuất file ảnh tổng hợp: Full_UAV_MEC_Report_2x4.png")