import os
import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import scipy
import pandas as pd
import math

# Set global style parameters
plt.rcParams.update({
    'font.size': 20,
    'axes.labelsize': 20,
    'axes.titlesize': 20,
    'xtick.labelsize': 20,
    'ytick.labelsize': 20,
    'legend.fontsize': 15,
    'axes.linewidth': 2,
    'grid.linewidth': 1,
    'lines.markersize': 9,
    'lines.markeredgewidth': 1.5
})

def filter_extreme_values(metric_values, lower_percentile=5, upper_percentile=90):
    if len(metric_values) == 0:
        return []
    lower_bound = np.percentile(metric_values, lower_percentile)
    upper_bound = np.percentile(metric_values, upper_percentile)

    return [value for value in metric_values if lower_bound <= value <= upper_bound]

@dataclass
class BandParameters:
    name: str
    frequency: float  # in GHz
    difs: float       # in seconds
    sifs: float       # in seconds
    n_obss: int       # number of OBSS
    p_error: float    # frame error probability

def get_contention_window_size(w_0, i, m, M):
    """Calculate the contention window size."""
    if i < m:
        w_i = w_0 * np.power(2, i)
    else:
        w_i = w_0 * np.power(2, m)
    return w_i

def calculate_lambda(lambda_poisson,p):
    return lambda_poisson#/(1-p)

def calculate_mu(mean_service_time):
    return 1/mean_service_time

def calculate_utilization_factor(lambda_total, mu, n_links):
    rho = lambda_total/(n_links*mu)
    #rho = np.clip(rho, 0.0001, 0.9999)
    return rho

def calculate_steady_state_0(rho, n_links):
    #rho = np.clip(rho, 0.0001, 0.9999)
    sum_1 = 0
    for k in range (n_links-1) :
        sum_1 += ((n_links*rho)**k)/math.factorial(k) 
    sum_2 = ((n_links*rho)**n_links)/(math.factorial(k)*(1-rho))
    rho_0 = (sum_1+sum_2)**(-1)
    return rho_0

def calculate_steady_state_k(rho,k, n_links):
    pi_0 = calculate_steady_state_0(rho, n_links)
    if k >= n_links :
        pi_k = pi_0*((rho**k)*(n_links**n_links))/math.factorial(n_links)
    else :
        pi_k = pi_0*((n_links*rho)**k)/math.factorial(k)
    return pi_k

def calculate_mean_service_time(p, p_collision, p_error, m, delta, slot_size, t_collision, t_error, mean_backoff_time, t_success):
    p = np.clip(p, 0.0001, 0.9999)
    mean_retransmission = max((p_collision*(1-np.power(p_collision,m))/(1-p_collision)),1/(1-p_error))
    mean_retransmission_time = (mean_backoff_time*slot_size)/(1-delta)+(p_collision*t_collision+(1-p_collision)*p_error*t_error)/p
    mean_transmission_time = (mean_backoff_time*slot_size)/(1-delta)+t_success

    mean_service_time = mean_retransmission*mean_retransmission_time + mean_transmission_time
    return mean_service_time

def calculate_mean_backoff_slot(p, m, M, w_0):
    mean_total_slot = 0
    for i in range(M):
        p_s = (1 - p) * np.power(p, i)
        mean_slot = 0
        for j in range(i + 1):
            w = (get_contention_window_size(w_0, i, m, M) - 1) / 2
            mean_slot += w
        mean_total_slot += p_s * mean_slot
    
    # mean_total_slot = (1-p-p*np.power(2*p,m))/(1-2*p)*(w_0 + 1) -1
    return mean_total_slot

def calculate_mean_backoff_time(rho, n_links, mean_backoff_slot):
    rho = np.clip(rho, 0.0001, 0.9999)
    sum_1 = 0
    for n in range (n_links-1) :
        sum_1 += calculate_steady_state_k(rho,n, n_links)*(mean_backoff_slot)/(n_links-n+1)

    sum_2 = (mean_backoff_slot/2)*calculate_steady_state_0(rho, n_links)*((n_links**n_links)/math.factorial(n_links))*((rho**n_links)/(1-rho))

    return sum_1 + sum_2

def calculate_delta(p_transmit, p_success, p_error, slot_size, t_success, t_collision, t_error):
    delta = 1 - slot_size/((1 - p_transmit) * slot_size + 
                  p_transmit * p_success * (1 - p_error) * t_success + 
                  p_transmit * (1 - p_success) * t_collision + p_transmit*p_success*p_error*t_error)
    return delta

def calculate_mean_waiting_time(rho, n_links, lambda_total):
    rho = np.clip(rho, 0.0001, 0.9999)
    p_busy = ((n_links*rho)**n_links)*calculate_steady_state_0(rho, n_links)/(math.factorial(n_links)*(1-rho))
    mean_queue = (rho*p_busy)/(1-rho)
    mean_waiting_time = mean_queue/lambda_total
    return mean_waiting_time

def calculate_collision_probability(tau, n_obss):
    """Calculate the conditional collision probability p."""
    return 1 - np.power(1 - tau, n_obss - 1)

def calculate_failure_probability(tau, n_obss, p_error):
    """Calculate the total failed transmission probability p."""
    p_coll = calculate_collision_probability(tau, n_obss)
    p_total = 1 - (1 - p_coll)*(1 - p_error)
    # p_total = np.clip(p_total, 0.0001, 0.9999)
    return p_total

def calculate_tau(rho, n_links, mean_backoff_time):
    """Calculate the transmission probability tau."""
    diff = 0
    for i in range (n_links-1) :
        diff += (n_links-i)/n_links*calculate_steady_state_k(rho,i, n_links)
    gamma = 1 - diff
    tau = gamma/(mean_backoff_time + 1)
    # tau = np.clip(tau, 0.0001, 0.9999)
    return tau

def update_value(alpha, old_value, new_value):
    """Update a value using exponential moving average."""
    updated_value = alpha * old_value + (1 - alpha) * new_value
    # updated_value = np.clip(updated_value, 0.0, 1.0)
    return updated_value

def calculate_transmission_probability(tau, n_obss):
    """Calculate the total probability of transmission."""
    return 1 - np.power(1 - tau, n_obss)

def calculate_success_probability(tau, n_obss):
    """Calculate the probability of successful transmission."""
    numerator = n_obss * tau * np.power(1 - tau, n_obss - 1)
    denominator = 1 - np.power(1 - tau, n_obss)
    return numerator / denominator

def calculate_success_time(phy_header, mac_header, packet_length, difs, sifs, ack, prop_delay):
    """Calculate the time for a successful transmission."""
    return phy_header + mac_header + packet_length + sifs + ack + difs + 2 * prop_delay

def calculate_collision_time(phy_header, mac_header, packet_length, difs, prop_delay):
    """Calculate the time for a collision."""
    return phy_header + mac_header + packet_length + difs + prop_delay

def calculate_throughput(n_obss, p_transmit, p_success, p_error, packet_length, data_rate, slot_size, t_success, t_collision, t_error, rho, mu, packet_size, n_links):
    """Calculate the throughput."""

    th = (1-p_error) * n_links * rho * mu * packet_size * 8
    return th

def calculate_reliability(p_coll,p_error, M, rho, n_links, n_error_links):
    """Calculate the reliability."""
    p_persistent = sum(calculate_steady_state_k(rho, k, n_links) / (n_links - k) for k in range(n_links + 1 - n_error_links) if (n_links - k) != 0)
    rel = (1 - ((p_coll)**(M+1)))*(1 - p_error*p_persistent)
    return rel

def calculate_lb_reliability(p_coll,p_error, M):
    """Calculate the reliability."""
    rel = (1 - ((p_coll)**(M+1)))*(1 - p_error)
    return rel

def calculate_delay(mean_service_time, mean_waiting_time):
    """Calculate the average delay."""
    delay = mean_service_time #+ mean_waiting_time
    return delay

def run_simulation_for_band(band_params, common_params):
    """Run the simulation for a specific frequency band."""
    # Unpack common parameters
    alpha = common_params['alpha']
    eps = common_params['eps']
    max_iter = common_params['max_iter']
    tau = common_params['tau']
    rho = common_params['rho']
    w_0 = common_params['w_0']
    m = common_params['m']
    M = common_params['M']
    n_links = common_params['n_links']
    data_rate = common_params['data_rate']
    min_data_rate = common_params['min_data_rate']
    lambda_poisson = common_params['lambda_poisson']
    phy_header = common_params['phy_header']
    mac_header_size = common_params['mac_header_size']
    ack_size = common_params['ack_size']
    slot_size = common_params['slot_size']
    prop_delay = common_params['prop_delay']
    packet_size = common_params['packet_size']
    n_error_links = common_params['n_error_links']
    
    # Calculate derived parameters
    mac_header = mac_header_size * 8 / data_rate
    ack = ack_size * 8 / min_data_rate
    packet_length = packet_size * 8 / data_rate
    
    t_success = calculate_success_time(phy_header, mac_header, packet_length, band_params.difs, band_params.sifs, ack, prop_delay)
    t_collision = calculate_collision_time(phy_header, mac_header, packet_length, band_params.difs, prop_delay)
    t_error = t_success

    # Iterative calculation
    for it in range(max_iter):
        p_collision = calculate_collision_probability(tau, band_params.n_obss)
        p_transmit = calculate_transmission_probability(tau, band_params.n_obss)
        p_success = calculate_success_probability(tau, band_params.n_obss)

        p = calculate_failure_probability(tau, band_params.n_obss, band_params.p_error)
        delta = calculate_delta(p_transmit, p_success, band_params.p_error, slot_size, t_success, t_collision, t_error)

        mean_backoff_slot = calculate_mean_backoff_slot(p, m, M, w_0)
        mean_backoff_time = calculate_mean_backoff_time(rho, n_links, mean_backoff_slot)

        mean_service_time = calculate_mean_service_time(p, p_collision, band_params.p_error, M, delta, slot_size, t_collision, t_error, mean_backoff_time, t_success)

        mu = calculate_mu(mean_service_time)
        lambda_total = calculate_lambda(lambda_poisson,p)
        
        rho_new = calculate_utilization_factor(lambda_total, mu, n_links)
        tau_new = calculate_tau(rho, n_links, mean_backoff_time)
        
        tau = update_value(alpha, tau, tau_new)
        rho = update_value(alpha, rho, rho_new)

        if abs(tau_new - tau) < eps and abs(rho_new - rho) < eps and tau >= 0 and tau <= 1 and rho >= 0:
            print('Last iteration ', it)
            break
    
    # Recalculate proabiilities and timings
    lambda_total = calculate_lambda(lambda_poisson,p)
    p_collision = calculate_collision_probability(tau, band_params.n_obss)
    p = calculate_failure_probability(tau, band_params.n_obss, band_params.p_error)
    print("Transmission probability (τ): ", tau)
    print("Collision probability (p): ", p_collision)
    p_transmit = calculate_transmission_probability(tau, band_params.n_obss)
    p_success = calculate_success_probability(tau, band_params.n_obss)
    mean_service_time = calculate_mean_service_time(p, p_collision, band_params.p_error, m, delta, slot_size, t_collision, t_error, mean_backoff_time, t_success)
    mu = calculate_mu(mean_service_time)
    mean_waiting_time = calculate_mean_waiting_time(rho, n_links, lambda_total)

    rho = calculate_utilization_factor(lambda_total, mu, n_links)
    print("Utiliyation factor (rho): ", rho)
    pi_k = calculate_steady_state_k(rho,n_links + 1, n_links)
    print("pi_k: ", pi_k)
    if rho > 1 or rho < 0:
        print('Warning!!! Queue is not in a steady state')


    # Calculate performance metrics
    stable_data_rate = min(packet_size * 8 * lambda_poisson,n_links*data_rate)
    throughput = calculate_throughput(band_params.n_obss, p_transmit, p_success, band_params.p_error, packet_length, data_rate, slot_size, t_success, t_collision, t_error, rho, mu, packet_size, n_links)
    reliability = calculate_reliability(p_collision, band_params.p_error,M, rho, n_links, n_error_links)
    delay = calculate_delay(mean_service_time, mean_waiting_time)
    
    return {
        'band': band_params.name,
        'frequency': band_params.frequency,
        'difs': band_params.difs,   
        'sifs': band_params.sifs,      
        'n_obss': band_params.n_obss,  
        'tau': tau,
        'p': p,
        'lambda': lambda_total,
        'mu' : mu,
        'delta' : delta,
        'rho' : rho,
        'p_collision' : p_collision,
        'p_transmit': p_transmit,
        'p_success': p_success,
        't_success': t_success,
        't_collision': t_collision,
        't_error' : t_error,
        'throughput': throughput,
        'reliability': reliability,
        'delay': delay
    }

def calculate_final_metrics(results):
    """Calculate aggregate metrics from all bands' simulation results."""
    # Initialize metrics
    total_throughput = 0
    reliability_sum = 0
    delay_sum = 0
    
    # Calculate metrics
    for result in results:
        # Throughput calculations
        total_throughput += result['throughput']
            
        # Reliability calculations
        reliability_sum += result['reliability']
        
        # Delay calculations
        delay_sum += result['delay']
    
    # Calculate averages
    num_bands = len(results)
    avg_reliability = reliability_sum / num_bands if num_bands > 0 else 0
    avg_delay = delay_sum / num_bands if num_bands > 0 else 0
    
    return {
        'total_throughput': total_throughput,
        'avg_reliability': avg_reliability,
        'avg_delay': avg_delay
    }

def run_analysis_for_n_obss(n_obss_values, bands_template, common_params):
    """Run simulations for different n_obss values and collect metrics."""
    result_summary = {
        'total_throughput': [],
        'avg_reliability': [],
        'avg_delay': []
    }
    
    for n in n_obss_values:
        # Create bands with current n_obss value
        bands = [
            BandParameters(name=b.name, frequency=b.frequency, 
                         difs=b.difs, sifs=b.sifs, n_obss=n, p_error=b.p_error)
            for b in bands_template
        ]
        
        # Run simulations
        results = [run_simulation_for_band(band, common_params) for band in bands]
        
        # Calculate aggregate metrics
        temp = calculate_final_metrics(results)
        
        # Store metrics
        result_summary['total_throughput'].append(temp['total_throughput'])
        result_summary['avg_reliability'].append(temp['avg_reliability'])
        result_summary['avg_delay'].append(temp['avg_delay'])
    
    return result_summary

def plot_throughput(n_obss_values, result_summary):
    """Plot throughput against n_obss values."""
    
    plt.figure(figsize=(8, 5))
    
    plt.plot(n_obss_values, [t/1e6 for t in result_summary['total_throughput']], 'bo-', label='WiFi 7 MLO - STR')

    plt.xlabel('Number of OBSS (n_obss)')
    plt.ylabel('Throughput (Mbps)')
    plt.title('Throughput vs Number of OBSS')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_reliability(n_obss_values, result_summary):
    """Plot reliability against n_obss values."""
    
    plt.figure(figsize=(8, 5))
    
    plt.plot(n_obss_values, [r*100 for r in result_summary['avg_reliability']], 'bo-', label='WiFi 7 MLO - STR')
    
    plt.xlabel('Number of OBSS (n_obss)')
    plt.ylabel('Reliability (%)')
    plt.title('Reliability vs Number of OBSS')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_delay(n_obss_values, result_summary):
    """Plot delay against n_obss values."""
    
    plt.figure(figsize=(8, 5))
    
    plt.plot(n_obss_values, [d*1e3 for d in result_summary['avg_delay']], 'bo-', label='WiFi 7 MLO - STR')
    
    plt.xlabel('Number of OBSS (n_obss)')
    plt.ylabel('Delay (ms)')
    plt.title('Delay vs Number of OBSS')
    plt.legend()
    plt.grid(True)
    plt.show()
    
def main():
    # Common parameters across all bands
    common_params = {
        'alpha': 0.5,
        'eps': 1e-6,
        'max_iter': 50000,
        'tau': 0.4,
        'rho': 0.4,
        'w_0': 16,
        'm': 6,
        'M': 7,
        'n_links' : 3,
        'lambda_poisson' : 500,
        'data_rate': 121.875e6,
        'min_data_rate': 7.25e6,
        'phy_header': 52e-6,
        'mac_header_size': 26,
        'ack_size': 14,
        'slot_size': 9e-6,
        'prop_delay': 67e-9,
        'packet_size': 1474,
        'n_error_links': 1
    }

    # Define parameters for different bands
    bands = [
        # BandParameters(name="2.4 GHz", frequency=2.4, difs=37e-6, sifs=10e-6, n_obss=3, p_error=0.0),
        # BandParameters(name="5 GHz", frequency=5.0, difs=43e-6, sifs=16e-6, n_obss=3, p_error=0.0),
        BandParameters(name="6 GHz", frequency=6.0, difs=43e-6, sifs=16e-6, n_obss=1, p_error=0.333)
    ]

    n_obss_values = list(range(1, 5))
    result_summary = run_analysis_for_n_obss(n_obss_values, bands, common_params)

    # %% Plotting for Wifi 7
    # Read the data file
    wifi_eht_input_filename = '../wifi-eht-static-scenario/wifi-eht-results-unsaturated-error.txt'

    base_name = os.path.basename(wifi_eht_input_filename)
    out_prefix = 'wifi8' if 'ehr' in base_name else 'wifi7' if 'eht' in base_name else 'wifi'
    reliability_str = '-reliability' if 'reliability' in base_name else ''

    wifi_eht = pd.read_csv(wifi_eht_input_filename, header=None, 
                    names=['seedNumber', 'numBSS', 'Throughput', 'PacketDropReliability', 
                            'PacketReceivedReliability', 'EndToEndDelay', 'ChannelAccessDelay'])

    # Clean the data (remove units from delay columns)
    wifi_eht['EndToEndDelay'] = wifi_eht['EndToEndDelay'].str.replace('ms', '').str.replace('+', '').astype(float)
    wifi_eht['ChannelAccessDelay'] = wifi_eht['ChannelAccessDelay'].str.replace('ms', '').str.replace('+', '').astype(float)

    # Set style for all plots
    plt.rcParams['figure.figsize'] = (10, 6)
    plt.rcParams['grid.color'] = '0.8'
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.linewidth'] = 0.5

    # Plot throughput (Simulation vs Analysis)
    th_data = [wifi_eht[wifi_eht['numBSS'] == val]['Throughput'].values for val in sorted(wifi_eht['numBSS'].unique())]
    positions = sorted(wifi_eht['numBSS'].unique())
    
    plt.figure()
    th_box = plt.violinplot(th_data, 
                      positions=positions,
                      vert=True,
                      widths=0.7,
                      showmeans=False,
                      showmedians=True,
                      showextrema=True)
    th_box_label = mpatches.Patch(facecolor='skyblue', edgecolor='navy',  
                          label='WiFi 7 MLO-STR (ns-3 Simulation)')
    th_analysis = plt.plot(n_obss_values, [t/1e6 for t in result_summary['total_throughput']], 'go', label='WiFi 7 MLO-STR (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=20)
    plt.ylabel('Throughput (Mbps)', fontsize=20)
    plt.grid(True, alpha=0.5)
    plt.xticks(n_obss_values)
    plt.tight_layout()
    plt.ylim(0, 30)
    plt.legend(handles=[th_box_label, th_analysis[0]], loc='best')
    
    plot_filename_th = f'../Graphs/frame-error-probability/th-{out_prefix}{reliability_str}-error.pdf'
    plt.savefig(plot_filename_th, dpi=1200)

    # Plot reliability (Simulation vs Analysis)
    rel_data = [wifi_eht[wifi_eht['numBSS'] == val]['PacketReceivedReliability'].values for val in sorted(wifi_eht['numBSS'].unique())]
    positions = sorted(wifi_eht['numBSS'].unique())

    plt.figure()
    rel_box = plt.violinplot(rel_data, 
                      positions=positions,
                      vert=True,
                      widths=0.7,
                      showmeans=False,
                      showmedians=True,
                      showextrema=True)
    rel_box_label = mpatches.Patch(facecolor='skyblue', edgecolor='navy',
                          label='WiFi 7 MLO-STR (ns-3 Simulation)')
    rel_analysis = plt.plot(n_obss_values, [r*100 for r in result_summary['avg_reliability']], 'go', label='WiFi 7 MLO-STR (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=20)
    plt.ylabel('Reliability (%)', fontsize=20)
    plt.grid(True, alpha=0.5)
    plt.xticks(n_obss_values)
    plt.tight_layout()
    plt.ylim(50, 105)
    plt.legend(handles=[rel_box_label, rel_analysis[0]], loc='best')
    
    plot_filename_rel = f'../Graphs/frame-error-probability/rel-{out_prefix}{reliability_str}-error.pdf'
    plt.savefig(plot_filename_rel, dpi=1200)

    # Plot delay (Simulation vs Analysis)
    delay_data = [filter_extreme_values(wifi_eht[wifi_eht['numBSS'] == val]['ChannelAccessDelay'].values) for val in sorted(wifi_eht['numBSS'].unique())]
    positions = sorted(wifi_eht['numBSS'].unique())

    plt.figure()
    delay_box = plt.violinplot(delay_data, 
                      positions=positions,
                      vert=True,
                      widths=0.7,
                      showmeans=False,
                      showmedians=True,
                      showextrema=True)
    delay_box_label = mpatches.Patch(facecolor='skyblue', edgecolor='navy',
                          label='WiFi 7 MLO-STR (ns-3 Simulation)')
    delay_analysis = plt.plot(n_obss_values, [d*1e3 for d in result_summary['avg_delay']], 'go', label='WiFi 7 MLO-STR (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=20)
    plt.ylabel('Delay (ms)', fontsize=20)
    plt.grid(True, alpha=0.5)
    plt.xticks(n_obss_values)
    plt.tight_layout()
    plt.ylim(0, 20)
    plt.legend(handles=[delay_box_label, delay_analysis[0]], loc='best')
    
    plot_filename_delay = f'../Graphs/frame-error-probability/ch-delay-{out_prefix}{reliability_str}-error.pdf'
    plt.savefig(plot_filename_delay, dpi=1200)

if __name__ == "__main__":
    main()