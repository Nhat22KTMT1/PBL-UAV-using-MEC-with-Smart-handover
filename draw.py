import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker
import os

# ================= CẤU HÌNH GIAO DIỆN =================
FONT_SIZE_MAIN_TITLE = 24
FONT_SIZE_SUBTITLE = 16
FONT_SIZE_AXIS_LABEL = 14
FONT_SIZE_TICK = 12
FONT_SIZE_LEGEND = 12
LINE_WIDTH = 2.5

SCENARIOS = {
    'Baseline': 'scenario1_baseline',
    'Smart (Load-Aware)': 'scenario1_smart'
}

COLORS = {'Baseline': '#d62728', 'Smart (Load-Aware)': '#1f77b4'} 
LINE_STYLES = {'Baseline': '--', 'Smart (Load-Aware)': '-'}

if not os.path.exists('graphs'): os.makedirs('graphs')

def read_csv(prefix, file_type):
    filenames = [f"scenario1{prefix}_{file_type}.csv", f"{prefix}_{file_type}.csv"]
    for fname in filenames:
        if os.path.exists(fname):
            try: return pd.read_csv(fname)
            except: pass
    return None

def configure_axis(ax, title, xlabel, ylabel):
    ax.set_title(title, fontweight='bold', fontsize=FONT_SIZE_SUBTITLE, pad=10)
    if xlabel: ax.set_xlabel(xlabel, fontsize=FONT_SIZE_AXIS_LABEL)
    ax.set_ylabel(ylabel, fontsize=FONT_SIZE_AXIS_LABEL)
    ax.tick_params(axis='both', which='major', labelsize=FONT_SIZE_TICK)
    ax.grid(True, linestyle='--', alpha=0.7, linewidth=0.8)

# ================= CÁC HÀM VẼ BIỂU ĐỒ =================

def plot_load_balancing(ax, mode='Baseline'):
    suffix = "_baseline" if mode == 'Baseline' else "_smart"
    df = read_csv(suffix, 'cellid')
    title = f'(A1) {mode}: Load Distribution' if mode == 'Baseline' else f'(A2) {mode}: Load Distribution'
    
    if df is not None and not df.empty:
        df = df[df['Time'] > 1.0]
        df = df[df['CellId'] > 0]
        df['Time_Sec'] = df['Time'].astype(int)
        load_data = df.groupby(['Time_Sec', 'CellId']).size().unstack(fill_value=0)
        for cell_id in [1, 2, 3]:
            if cell_id in load_data.columns:
                style = '--' if mode == 'Baseline' else '-'
                ax.plot(load_data.index.to_numpy(), load_data[cell_id].to_numpy(), 
                       label=f'UAV {cell_id}', linewidth=LINE_WIDTH, linestyle=style)
        
        # Ngưỡng Load = 6 UE
        ax.axhline(y=6, color='r', linestyle=':', label='Threshold (6)', alpha=0.8, linewidth=2)
        
    configure_axis(ax, title, None, 'Active UEs')
    ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
    ax.legend(fontsize=FONT_SIZE_LEGEND, loc='upper right')

def plot_handover_cumulative(ax):
    for name, suffix in SCENARIOS.items():
        file_suffix = "_baseline" if "Baseline" in name else "_smart"
        df = read_csv(file_suffix, 'handover')
        if df is not None and not df.empty:
            df = df.sort_values('Time')
            ax.plot(df['Time'].to_numpy(), df['HandoverCount'].to_numpy(), 
                   label=name, color=COLORS[name], linestyle=LINE_STYLES[name], linewidth=LINE_WIDTH)
    configure_axis(ax, '(B) Cumulative Handover Count', None, 'Total Handovers')
    ax.legend(fontsize=FONT_SIZE_LEGEND)

def plot_latency(ax):
    for name, suffix in SCENARIOS.items():
        file_suffix = "_baseline" if "Baseline" in name else "_smart"
        df = read_csv(file_suffix, 'mec_offload')
        if df is not None and not df.empty:
            df['Time_Int'] = df['Time'].astype(int)
            mean_val = df['Latency'].mean()
            multiplier = 1000.0 if mean_val < 5.0 else 1.0 
            avg_latency = df.groupby('Time_Int')['Latency'].mean() * multiplier
            avg_latency_smooth = avg_latency.rolling(window=5, min_periods=1).mean()
            ax.plot(avg_latency_smooth.index.to_numpy(), avg_latency_smooth.to_numpy(), 
                    label=name, color=COLORS[name], linestyle=LINE_STYLES[name], linewidth=LINE_WIDTH)
    configure_axis(ax, '(C) Average Task Latency', None, 'Latency (ms)')
    ax.legend(fontsize=FONT_SIZE_LEGEND)

def plot_energy(ax):
    for name, suffix in SCENARIOS.items():
        file_suffix = "_baseline" if "Baseline" in name else "_smart"
        df = read_csv(file_suffix, 'uav_energy')
        if df is not None and not df.empty:
            total = df.pivot_table(index='Time', columns='UAV_ID', values='ConsumedEnergy_J').sum(axis=1)
            ax.plot(total.index.to_numpy(), total.values, 
                    label=name, color=COLORS[name], linestyle=LINE_STYLES[name], linewidth=LINE_WIDTH)
    configure_axis(ax, '(D) Total UAV Energy Consumption', None, 'Energy (Joule)')
    ax.legend(fontsize=FONT_SIZE_LEGEND)

def plot_pkt_loss(ax):
    names, values = [], []
    for name, suffix in SCENARIOS.items():
        file_suffix = "_baseline" if "Baseline" in name else "_smart"
        df = read_csv(file_suffix, 'flow_stats')
        if df is not None and 'LossRate' in df.columns:
            short_name = "Smart" if "Smart" in name else "Base"
            names.append(short_name)
            values.append(df['LossRate'].mean())
    
    if names:
        bars = ax.bar(names, values, color=[COLORS['Baseline'], COLORS['Smart (Load-Aware)']], 
                     width=0.5, edgecolor='black', linewidth=1.2)
        for bar in bars:
            yval = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2, yval + 0.5, f'{yval:.2f}%', 
                   ha='center', va='bottom', fontsize=FONT_SIZE_TICK, fontweight='bold')
            
    configure_axis(ax, '(E) Avg Packet Loss', None, 'Loss Rate (%)')
    if values: ax.set_ylim(0, max(values) * 1.3)

def draw_parameter_table(ax):
    ax.axis('off')
    ax.set_title('(F) Simulation Parameters', fontweight='bold', fontsize=FONT_SIZE_SUBTITLE, pad=20)
    
    # Bảng tham số với ngưỡng Load 
    table_data = [
        ["Parameter", "Value"],
        ["Simulation Time", "99.0 s"],
        ["Number of UEs / UAVs", "15 UEs / 3 UAVs"],
        ["App Traffic Type", "UDP Video (Downlink)"],
        ["Traffic Rate", "4.1 Mbps / UE"],
        ["LTE Bandwidth", "20 MHz (100 RBs)"], 
        ["MEC Processing", "Freq Sharing (Eq. 8)"],
        ["Handover Logic", "A3-RSRP vs. Load-Aware"],
        ["Load Threshold", "6 UEs / Cell"], 
        ["Pathloss Model", "LogDistance (RefLoss=30)"]
    ]
    
    table = ax.table(cellText=table_data, loc='center', cellLoc='left', colWidths=[0.45, 0.55])
    table.auto_set_font_size(False)
    table.set_fontsize(13)
    table.scale(1, 2.2) 
    
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('black')
        if row == 0:
            cell.set_text_props(weight='bold', color='white')
            cell.set_facecolor('#404040') 
        else:
            cell.set_facecolor('#f5f5f5' if row % 2 == 0 else 'white')

# ================= MAIN PROGRAM =================
if __name__ == "__main__":
    fig, axes = plt.subplots(4, 2, figsize=(16, 22)) 
    axs = axes.flatten()
    
    print(">>> Đang vẽ biểu đồ với Layout mới (Bảng ở cuối)...")
    
    try:
        # Hàng 1
        plot_load_balancing(axs[0], mode='Baseline') 
        plot_load_balancing(axs[1], mode='Smart (Load-Aware)')    
        
        # Hàng 2
        plot_handover_cumulative(axs[2])     
        plot_latency(axs[3])                 
        
        # Hàng 3 
        plot_energy(axs[4])                  
        plot_pkt_loss(axs[5])
        
        # Hàng 4
        axs[6].axis('off') 
        draw_parameter_table(axs[7])
        
        plt.suptitle("PERFORMANCE ANALYSIS: BASELINE vs. SMART (LOAD-AWARE)\nUAV-MEC NETWORK", 
                     fontsize=FONT_SIZE_MAIN_TITLE, fontweight='bold', y=0.995)
        
        plt.tight_layout(rect=[0, 0, 1, 0.985], h_pad=3.0, w_pad=2.0)
        
        output_file = 'graphs/bieudo_final_v3.png'
        plt.savefig(output_file, dpi=200, bbox_inches='tight')
        print(f" Xong! Ảnh đã lưu tại: {output_file}")
        
    except Exception as e:
        print(f" Có lỗi xảy ra: {e}")
        import traceback
        traceback.print_exc()
