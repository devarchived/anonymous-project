import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import scipy
import pandas as pd
import seaborn as sns

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

def calculate_collision_probability(tau, n_obss):
    """Calculate the conditional collision probability p."""
    return 1 - np.power(1 - tau, n_obss - 1)

def calculate_failure_probability(tau, n_obss, p_error):
    """Calculate the total failed transmission probability p."""
    p_coll = calculate_collision_probability(tau, n_obss)
    return 1 - (1 - p_coll)*(1 - p_error) 

def calculate_tau(p, tau, w_0, m, M, n_obss, p_f):
    """Calculate the transmission probability tau."""
    numerator = 1 - p**(M+1)

    denominator_sum = 0
    for i in range (M+1):
        inner_sum = 0
        w_i = get_contention_window_size(w_0, i, m, M)
        for j in range (1,w_i) :
            inner_sum += (w_i - j) / w_i
        temp = (1 + (1 / (1 - p_f)) * inner_sum) * (p**i)
        denominator_sum += temp

    denominator = denominator_sum*(1-p)

    # if abs(denominator) < 1e-10:
    #     return 0.5

    tau_new = numerator / denominator
    return tau_new

def calculate_p_ei(tau, n_obss):
    p_ei = (1-tau)**(n_obss-1)
    return p_ei

def calculate_p_es(tau, n_obss):
    p_es = scipy.special.comb(n_obss-1, 1)*tau*((1-tau)**(n_obss-2))
    return p_es

def calculate_p_ec(p_ei, p_es):
    p_ec = 1 - p_ei - p_es
    return p_ec

def calculate_p_ss(w_0):
    p_ss = 1/w_0
    return p_ss

def calculate_p_si(p_ss):
    p_si = 1 - p_ss
    return p_si

def calculate_average_backoff_window(w_0, m, M, p):
    p_dc = p**(M*1)
    
    numerator_sum = 0
    for i in range(M+1):
        numerator_sum += (1 -p)*(p**i)*get_contention_window_size(w_0, i, m, M)

    mean_cw = numerator_sum / (1 - p_dc)
    return mean_cw

def calculate_pmf_n (n_obss, n, tau):
    pmf_n = scipy.special.comb(n_obss-1, n)*tau**n*(1-tau)**(n_obss-1-n)
    return pmf_n

def calculate_p_ci(n_obss, tau, mean_cw):
    if n_obss > 2 :
        p_ci = sum(calculate_pmf_n(n_obss, n, tau) * (1 - 1/mean_cw)**n for n in range(2, n_obss))
    else : 
        p_ci = 1
    return p_ci

def calculate_p_cs(n_obss, tau, mean_cw):
    if n_obss > 2 :
        p_cs = sum(calculate_pmf_n(n_obss, n, tau) * n * (1/mean_cw) * (1 - 1/mean_cw)**(n-1) for n in range(2, n_obss))
    else : 
        p_cs = 0
    return p_cs

def calculate_p_cc(n_obss, p_ci, p_cs):
    p_cc = 1 - p_ci - p_cs
    return p_cc

def calculate_p_S(p_ss, p_es, p_cs):
    p_S = p_ss + p_es + p_cs
    return p_S

def calculate_p_C(p_cc, p_ec):
    p_C = p_cc + p_ec
    return p_C

def calculate_p_I(p_ei, p_ci, p_si):
    p_I = p_ei + p_ci + p_si
    return p_I

def calculate_decrement_probability(p_I):
    p_d = p_I
    return p_d

def calculate_freezing_probability(p_d):
    p_f = 1 - p_d
    return p_f

def update_value(alpha, old_value, new_value):
    """Update a value using exponential moving average."""
    return alpha * old_value + (1 - alpha) * new_value

def calculate_transmission_probability(tau, n_obss):
    """Calculate the probability of transmission."""
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

def calculate_throughput(n_obss, p_transmit, p_success, p_error, packet_length, data_rate, slot_size, t_success, t_collision, t_error):
    """Calculate the throughput."""
    numerator = p_transmit * p_success * (1 - p_error) * packet_length * data_rate * 1/n_obss
    denominator = ((1 - p_transmit) * slot_size + 
                  p_transmit * p_success * (1 - p_error) * t_success + 
                  p_transmit * (1 - p_success) * t_collision + p_transmit*p_success*p_error*t_error)
    return numerator / denominator

def calculate_first_transmission_reliability(tau, p, n_obss):
    """Calculate the reliability."""
    return 0

def calculate_reliability(p, M):
    """Calculate the reliability."""
    return 1 - np.power(p, M+1)

def calculate_t_I (slot_size):
    return slot_size

def calculate_t_S (p_ss, t_success, t_I):
    t_S = 1/(1-p_ss)*t_success + t_I
    return t_S

def calculate_t_C (p, p_cs, p_ci, p_cc, t_collision, t_S, t_I, M):
    p_cc = np.clip(p_cc, 0, p)

    sum_coll_time = 0
    for i in range(M+1):
        sum_coll_time += i*(p_cc)**i

    sum1 = sum_coll_time*t_collision
    sum2 = (p_cs/(1-p_cc)) * t_S
    sum3 = (p_ci/(1-p_cc)) * t_I
    
    t_C = sum1 + sum2 + sum3

    return t_C

def calculate_sigma_b(p_ei, p_es, p_ec, p_ss, p_si, p_ci, p_cs, p_cc, t_I, t_S, t_C, p_d):
    sigma_b = ((p_ei*t_I) + (p_es*t_S) + (p_ec*t_C))/p_d
    return sigma_b

def calculate_sigma_t(p_ei, p_es, p_ec, p_ss, p_si, p_ci, p_cs, p_cc, t_I, t_S, t_C, mean_cw):
    sigma_t = ((p_ei*t_I) + (p_es*t_S) + (p_ec*t_C))*(1-1/mean_cw)
    return sigma_t

def calculate_sigma(tau, sigma_b, sigma_t):
    sigma = (1-tau)*sigma_b + tau*sigma_t
    return sigma

def calculate_delay(p, p_collision, p_error, m, w_0, sigma, t_success, t_collision, t_error):
    """Calculate the average delay."""
    p = np.clip(p, 0.0001, 0.9999)
    t_backoff = 0
    
    for i in range(m):
        p_s = (1 - p) * np.power(p, i)
        mean_t_slot = 0
        for j in range(i + 1):
            w = (np.power(2, j) * w_0 - 1) / 2
            mean_t_slot += w * sigma
        t_backoff += p_s * (t_success + i * (p_collision*t_collision + (1-p_collision)*p_error*t_error)/p + mean_t_slot)
    
    delay = t_backoff / (1 - np.power(p, m + 1))
    return delay

def run_simulation_for_band(band_params, common_params):
    """Run the simulation for a specific frequency band."""
    # Unpack common parameters
    alpha = common_params['alpha']
    eps = common_params['eps']
    max_iter = common_params['max_iter']
    tau = common_params['tau']
    w_0 = common_params['w_0']
    m = common_params['m']
    M = common_params['M']
    data_rate = common_params['data_rate']
    min_data_rate = common_params['min_data_rate']
    lambda_poisson = common_params['lambda_poisson']
    phy_header = common_params['phy_header']
    mac_header_size = common_params['mac_header_size']
    ack_size = common_params['ack_size']
    slot_size = common_params['slot_size']
    prop_delay = common_params['prop_delay']
    packet_size = common_params['packet_size']
    
    # Calculate derived parameters
    mac_header = mac_header_size * 8 / data_rate
    print("mac_header", mac_header)
    ack = ack_size * 8 / min_data_rate
    packet_length = packet_size * 8 / data_rate
    
    # Iterative calculation
    for it in range(max_iter):
        p = calculate_failure_probability(tau, band_params.n_obss, band_params.p_error)
        
        p_ei = calculate_p_ei(tau, band_params.n_obss)
        p_es = calculate_p_es(tau, band_params.n_obss)
        p_ec = calculate_p_ec(p_ei, p_es)
        p_ss = calculate_p_ss(w_0)
        p_si = calculate_p_si(p_ss)

        mean_cw = calculate_average_backoff_window(w_0, m, M, p)
        p_ci = calculate_p_ci(band_params.n_obss, tau, mean_cw)
        p_cs = calculate_p_cs(band_params.n_obss, tau, mean_cw)
        p_cc = calculate_p_cc(band_params.n_obss, p_ci, p_cs)

        p_S = calculate_p_S(p_ss, p_es, p_cs)
        p_C = calculate_p_C(p_cc, p_ec)
        p_I = calculate_p_I(p_ei, p_ci, p_si)

        p_d = calculate_decrement_probability(p_I)
        p_f = calculate_freezing_probability(p_d)

        tau_new = calculate_tau(p, tau, w_0, m, M, band_params.n_obss, p_f)
        tau = update_value(alpha, tau, tau_new)
        
        if (
            abs(tau_new - tau) < eps and
            0 < tau_new < 1 and
            0 < tau < 1
        ):
            print('Last iteration ', it)
            print("Transmission probability (τ): ", tau)
            print("Collision probability (p): ", p)
            break
    
    # Account the channel error probability

    print("Transmission probability (τ): ", tau)
    print("Collision probability (p): ", p)

    # Calculate state probabilities
    p_collision = calculate_collision_probability(tau, band_params.n_obss)
    print("Collision probability (p_collision): ", p_collision)
    p_transmit = calculate_transmission_probability(tau, band_params.n_obss)
    print("Transmission probability (p_transmit): ", p_transmit)
    p_success = calculate_success_probability(tau, band_params.n_obss)
    print("Success probability (p_success): ", p_success)

    # Calculate timing parameters
    t_success = calculate_success_time(phy_header, mac_header, packet_length, 
                                      band_params.difs, band_params.sifs, ack, prop_delay)
    t_collision = calculate_collision_time(phy_header, mac_header, packet_length, 
                                         band_params.difs, prop_delay)
    t_error = t_success
    t_I = calculate_t_I (slot_size)
    t_S = calculate_t_S (p_ss, t_success, t_I)
    t_C = calculate_t_C (p, p_cs, p_ci, p_cc, t_collision, t_S, t_I, M)
    
    # Calculate throughput
    throughput = calculate_throughput(band_params.n_obss, p_transmit, p_success, band_params.p_error, packet_length, data_rate, slot_size, t_success, t_collision, t_error)
    
    # Calculate packet drop reliability
    reliability = calculate_reliability(p, M)

    # Calculate channel access delay
    sigma_b = calculate_sigma_b(p_ei, p_es, p_ec, p_ss, p_si, p_ci, p_cs, p_cc, t_I, t_S, t_C, p_d)
    sigma_t = calculate_sigma_t(p_ei, p_es, p_ec, p_ss, p_si, p_ci, p_cs, p_cc, t_I, t_S, t_C, mean_cw)
    sigma = calculate_sigma(tau, sigma_b, sigma_t)

    delay = calculate_delay(p, p_collision, band_params.p_error, m, w_0, sigma, t_success, t_collision, t_error)
    print(delay)
    
    return {
        'band': band_params.name,
        'frequency': band_params.frequency,
        'difs': band_params.difs,   
        'sifs': band_params.sifs,      
        'n_obss': band_params.n_obss,  
        'tau': tau,
        'p': p,
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
    max_throughput = 0
    reliability_sum = 0
    reliability_prod = 1
    delay_sum = 0
    min_delay = float('inf')
    
    # Calculate metrics
    for result in results:
        # Throughput calculations
        total_throughput += result['throughput']
        if result['throughput'] > max_throughput:
            max_throughput = result['throughput']
            
        # Reliability calculations
        reliability_sum += result['reliability']
        reliability_prod *= (1-result['reliability'])
        
        # Delay calculations
        delay_sum += result['delay']
        if result['delay'] < min_delay:
            min_delay = result['delay']
    
    # Calculate averages
    num_bands = len(results)
    print("Debugging num_band : ", num_bands)
    avg_reliability = reliability_sum / num_bands if num_bands > 0 else 0
    print("Debugging avg_reliability : ", avg_reliability)
    avg_delay = delay_sum / num_bands if num_bands > 0 else 0
    
    # Calculate reliability for JT reliability mode
    jt_reliability = 1 - reliability_prod

    return {
        'total_throughput': total_throughput,
        'max_throughput': max_throughput,
        'avg_reliability': avg_reliability,
        'jt_reliability' : jt_reliability,
        'avg_delay': avg_delay,
        'min_delay': min_delay
    }

def run_analysis_for_n_obss(n_obss_values, bands_template, common_params):
    """Run simulations for different n_obss values and collect metrics."""
    result_summary = {
        'total_throughput': [],
        'max_throughput': [],
        'avg_reliability': [],
        'jt_reliability' : [],
        'first_transmission_reliability' : [],
        'jt_first_transmission_reliability' : [],
        'avg_delay': [],
        'min_delay': []
    }
    
    for n in n_obss_values:
        # Create bands with current n_obss value
        print("Debugging n_obss: ", n)
        bands = [
            BandParameters(name=b.name, frequency=b.frequency, 
                         difs=b.difs, sifs=b.sifs, n_obss=n, p_error=b.p_error)
            for b in bands_template
        ]
        
        # Run simulations
        results = [run_simulation_for_band(band, common_params) for band in bands]
        
        # Calculate aggregate metrics
        temp = calculate_final_metrics(results)

        # Calculate first_transmission_reliability
        first_transmission_reliability_sum = 0
        for result in results:
            first_transmission_reliability_sum += (1 - result['p'])
        first_transmission_reliability = first_transmission_reliability_sum / len(results)

        # Calculate jt_first_transmission_reliability
        total_first_transmission_reliability = 1
        for result in results:
            total_first_transmission_reliability *= result['p']
        jt_first_transmission_reliability = 1 - total_first_transmission_reliability

        # Store metrics
        result_summary['total_throughput'].append(temp['total_throughput'])
        result_summary['max_throughput'].append(temp['max_throughput'])
        result_summary['avg_reliability'].append(temp['avg_reliability'])
        result_summary['jt_reliability'].append(temp['jt_reliability'])
        result_summary['first_transmission_reliability'].append(first_transmission_reliability)
        result_summary['jt_first_transmission_reliability'].append(jt_first_transmission_reliability)
        result_summary['avg_delay'].append(temp['avg_delay'])
        result_summary['min_delay'].append(temp['min_delay'])
    
    return result_summary

def plot_throughput(n_obss_values, result_summary):
    """Plot throughput against n_obss values."""
    
    plt.figure(figsize=(8, 5))
    
    plt.plot(n_obss_values, [t/1e6 for t in result_summary['total_throughput']], 'bo-', label='WiFi 8 JT Th Mode')
    plt.plot(n_obss_values, [t/1e6 for t in result_summary['max_throughput']], 'go-', label='WiFi 8 JT Rel Mode')
    plt.plot(n_obss_values, [t/1e6 for t in result_summary['total_throughput']], 'r+--', label='WiFi 7 MLO-STR')

    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Throughput (Mbps)', fontsize=12)
    plt.ylim(0, 150)
    plt.legend()
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.show()

def plot_reliability(n_obss_values, result_summary):
    """Plot reliability against n_obss values."""
    
    plt.figure(figsize=(8, 5))
    
    plt.plot(n_obss_values, [r*100 for r in result_summary['avg_reliability']], 'bo-', label='WiFi 8 JT Th Mode')
    plt.plot(n_obss_values, [r*100 for r in result_summary['jt_reliability']], 'go-', label='WiFi 8 JT Rel Mode')
    plt.plot(n_obss_values, [r*100 for r in result_summary['avg_reliability']], 'r+--', label='WiFi 7 MLO-STR')

    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Reliability (%)', fontsize=12)
    plt.ylim(50,105)
    plt.legend()
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.show()

def plot_first_trans_reliability(n_obss_values, result_summary):
    """Plot reliability against n_obss values."""
    
    plt.figure(figsize=(8, 5))
    
    plt.plot(n_obss_values, [r*100 for r in result_summary['first_transmission_reliability']], 'bo-', label='WiFi 8 JT Th Mode')
    plt.plot(n_obss_values, [r*100 for r in result_summary['jt_first_transmission_reliability']], 'go-', label='WiFi 8 JT Rel Mode')
    plt.plot(n_obss_values, [r*100 for r in result_summary['first_transmission_reliability']], 'r+--', label='WiFi 7 MLO-STR')

    
    plt.xlabel('Number of OBSS (n_obss)')
    plt.ylabel('Reliability (%)')
    plt.title('First Transmission Reliability vs Number of OBSS')
    plt.ylim(50,105)
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_delay(n_obss_values, result_summary):
    """Plot delay against n_obss values."""
    
    plt.figure(figsize=(8, 5))
    
    plt.plot(n_obss_values, [d*1e3 for d in result_summary['avg_delay']], 'bo-', label='WiFi 8 JT Th Mode')
    plt.plot(n_obss_values, [d*1e3 for d in result_summary['min_delay']], 'go-', label='WiFi 8 JT Rel Mode')
    plt.plot(n_obss_values, [d*1e3 for d in result_summary['avg_delay']], 'r+--', label='WiFi 7 MLO-STR')

    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Delay (ms)', fontsize=12)
    plt.ylim(0, 10)
    plt.legend()
    plt.tight_layout()
    plt.grid(True, alpha=0.5)
    plt.show()

def main():
    # Run the simulation from wifi-ekici.py
    common_params = {
        'alpha': 0.5,
        'eps': 1e-10,
        'max_iter': 10000,
        'tau': 0.2,
        'w_0': 16,
        'm': 6,
        'M': 7,
        'lambda_poisson' : 1000,
        'data_rate': 121.875e6,
        'min_data_rate': 7.25e6,
        'phy_header': 48e-6,
        'mac_header_size': 26,
        'ack_size': 14,
        'slot_size': 9e-6,
        'prop_delay': 67e-9,
        'packet_size': 1474
    }
    
    error_scenarios = [
        {"2.4 GHz": 0.999, "5 GHz": 0.0, "6 GHz": 0.0},  # Only 2.4 GHz has errors
        {"2.4 GHz": 0.0, "5 GHz": 0.999, "6 GHz": 0.0},  # Only 5 GHz has errors
        {"2.4 GHz": 0.0, "5 GHz": 0.0, "6 GHz": 0.999}   # Only 6 GHz has errors
    ]

    n_obss_values = list(range(1, 5))

    all_results = []

    for scenario in error_scenarios:
        print(f"\nRunning scenario with p_error: {scenario}")
        
        bands = [
            BandParameters(name="2.4 GHz", frequency=2.4, difs=37e-6, sifs=10e-6, n_obss=4, p_error=scenario["2.4 GHz"]),
            BandParameters(name="5 GHz", frequency=5.0, difs=43e-6, sifs=16e-6, n_obss=4, p_error=scenario["5 GHz"]),
            BandParameters(name="6 GHz", frequency=6.0, difs=43e-6, sifs=16e-6, n_obss=4, p_error=scenario["6 GHz"])
        ]
        
        # Run simulations
        result_summary = run_analysis_for_n_obss(n_obss_values, bands, common_params)
        all_results.append(result_summary)
    
    # Calculate average results across all scenarios
    avg_results = {
        'total_throughput': np.mean([r['total_throughput'] for r in all_results], axis=0),
        'max_throughput': np.mean([r['max_throughput'] for r in all_results], axis=0),
        'avg_reliability': np.mean([r['avg_reliability'] for r in all_results], axis=0),
        'jt_reliability': np.mean([r['jt_reliability'] for r in all_results], axis=0),
        'first_transmission_reliability': np.mean([r['first_transmission_reliability'] for r in all_results], axis=0),
        'jt_first_transmission_reliability': np.mean([r['jt_first_transmission_reliability'] for r in all_results], axis=0),
        'avg_delay': np.mean([r['avg_delay'] for r in all_results], axis=0),
        'min_delay': np.mean([r['min_delay'] for r in all_results], axis=0)
    }

    # %% Plotting for Wifi 8
    # Read the data file
    wifi_ehr = pd.read_csv('../wifi-ehr-static-scenario/wifi-ehr-results.txt', header=None, 
                    names=['seedNumber', 'numBSS', 'Throughput', 'PacketDropReliability', 
                            'PacketReceivedReliability', 'EndToEndDelay', 'ChannelAccessDelay'])

    # Clean the data (remove units from delay columns)
    wifi_ehr['EndToEndDelay'] = wifi_ehr['EndToEndDelay'].str.replace('ms', '').str.replace('+', '').astype(float)
    wifi_ehr['ChannelAccessDelay'] = wifi_ehr['ChannelAccessDelay'].str.replace('ms', '').str.replace('+', '').astype(float)

    # Set style for all plots
    plt.rcParams['figure.figsize'] = (10, 6)
    plt.rcParams['grid.color'] = '0.8'
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.linewidth'] = 0.5

    # Plot throughput (Simulation vs Analysis)
    th_data = [wifi_ehr[wifi_ehr['numBSS'] == val]['Throughput'].values for val in sorted(wifi_ehr['numBSS'].unique())]
    positions = sorted(wifi_ehr['numBSS'].unique())
    
    plt.figure()#dpi=150)
    th_box = plt.boxplot(th_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                      showfliers=True,
                      medianprops=dict(color="red"))
    th_box_label = mpatches.Patch(facecolor='white', edgecolor='black', 
                          label='WiFi 8 JT (ns-3 Simulation)')
    th_analysis = plt.plot(n_obss_values, [t/1e6 for t in avg_results['total_throughput']], 'go-', label='WiFi 8 JT (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Throughput (Mbps)', fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 150)
    plt.legend(handles=[th_box_label, th_analysis[0]], loc='best')
    plt.show()

    # Plot reliability (Simulation vs Analysis)
    rel_data = [wifi_ehr[wifi_ehr['numBSS'] == val]['PacketDropReliability'].values for val in sorted(wifi_ehr['numBSS'].unique())]
    positions = sorted(wifi_ehr['numBSS'].unique())

    plt.figure()#dpi=150)
    rel_box = plt.boxplot(rel_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                      showfliers=True,
                      medianprops=dict(color="red"))
    rel_box_label = mpatches.Patch(facecolor='white', edgecolor='black', 
                          label='WiFi 8 JT (ns-3 Simulation)')
    rel_analysis = plt.plot(n_obss_values, [r*100 for r in avg_results['avg_reliability']], 'go-', label='WiFi 8 JT (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Reliability (%)', fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(50,105)
    plt.legend(handles=[rel_box_label, rel_analysis[0]], loc='best')
    plt.show()

    # Plot delay (Simulation vs Analysis)
    delay_data = [wifi_ehr[wifi_ehr['numBSS'] == val]['ChannelAccessDelay'].values for val in sorted(wifi_ehr['numBSS'].unique())]
    positions = sorted(wifi_ehr['numBSS'].unique())

    plt.figure()#dpi=150)
    delay_box = plt.boxplot(delay_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                      showfliers=True,
                      medianprops=dict(color="red"))
    delay_box_label = mpatches.Patch(facecolor='white', edgecolor='black', 
                          label='WiFi 8 JT (ns-3 Simulation)')
    delay_analysis = plt.plot(n_obss_values, [d*1e3 for d in avg_results['avg_delay']], 'go-', label='WiFi 8 JT (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Delay (ms)', fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 5)
    plt.legend(handles=[delay_box_label, delay_analysis[0]], loc='best')
    plt.show()

    # %% Plotting for Wifi 8 reliability mode
    # Read the data file
    wifi_ehr_rel = pd.read_csv('../wifi-ehr-static-scenario/sorted-wifi-ehr-results-reliability.txt', header=None, 
                    names=['seedNumber', 'numBSS', 'Throughput', 'PacketDropReliability', 
                            'PacketReceivedReliability', 'EndToEndDelay', 'ChannelAccessDelay'])

    # Clean the data (remove units from delay columns)
    wifi_ehr_rel['EndToEndDelay'] = wifi_ehr_rel['EndToEndDelay'].str.replace('ms', '').str.replace('+', '').astype(float)
    wifi_ehr_rel['ChannelAccessDelay'] = wifi_ehr_rel['ChannelAccessDelay'].str.replace('ms', '').str.replace('+', '').astype(float)

    # Set style for all plots
    plt.rcParams['figure.figsize'] = (10, 6)
    plt.rcParams['grid.color'] = '0.8'
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.linewidth'] = 0.5

    # Plot throughput (Simulation vs Analysis)
    th_data = [wifi_ehr_rel[wifi_ehr_rel['numBSS'] == val]['Throughput'].values for val in sorted(wifi_ehr_rel['numBSS'].unique())]
    positions = sorted(wifi_ehr_rel['numBSS'].unique())
    
    plt.figure()#dpi=150)
    th_box = plt.boxplot(th_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                      showfliers=True,
                      medianprops=dict(color="red"))
    th_box_label = mpatches.Patch(facecolor='white', edgecolor='black', 
                          label='WiFi 8 JT Reliability Mode (ns-3 Simulation)')
    th_analysis = plt.plot(n_obss_values, [t/1e6 for t in avg_results['max_throughput']], 'go-', label='WiFi 8 JT Reliability Mode (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Throughput (Mbps)', fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 150)
    plt.legend(handles=[th_box_label, th_analysis[0]], loc='best')
    plt.show()

    # Plot reliability (Simulation vs Analysis)
    rel_data = [wifi_ehr_rel[wifi_ehr_rel['numBSS'] == val]['PacketDropReliability'].values for val in sorted(wifi_ehr_rel['numBSS'].unique())]
    positions = sorted(wifi_ehr_rel['numBSS'].unique())

    plt.figure()#dpi=150)
    rel_box = plt.boxplot(rel_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                      showfliers=True,
                      medianprops=dict(color="red"))
    rel_box_label = mpatches.Patch(facecolor='white', edgecolor='black', 
                          label='WiFi 8 JT Reliability Mode (ns-3 Simulation)')
    rel_analysis = plt.plot(n_obss_values, [r*100 for r in avg_results['jt_reliability']], 'go-', label='WiFi 8 JT Reliability Mode (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Reliability (%)', fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(50, 105)
    plt.legend(handles=[rel_box_label, rel_analysis[0]], loc='best')
    plt.show()

    # Plot delay (Simulation vs Analysis)
    delay_data = [wifi_ehr_rel[wifi_ehr_rel['numBSS'] == val]['ChannelAccessDelay'].values for val in sorted(wifi_ehr_rel['numBSS'].unique())]
    positions = sorted(wifi_ehr_rel['numBSS'].unique())

    plt.figure()#dpi=150)
    delay_box = plt.boxplot(delay_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                      showfliers=True,
                      medianprops=dict(color="red"))
    delay_box_label = mpatches.Patch(facecolor='white', edgecolor='black', 
                          label='WiFi 8 JT Reliability Mode (ns-3 Simulation)')
    delay_analysis = plt.plot(n_obss_values, [d*1e3 for d in avg_results['min_delay']], 'go-', label='WiFi 8 JT Reliability Mode (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Delay (ms)', fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 5)
    plt.legend(handles=[delay_box_label, delay_analysis[0]], loc='best')
    plt.show()

    # %% Plotting for Wifi 7
    # Read the data file
    wifi_eht = pd.read_csv('../wifi-eht-static-scenario/sorted-wifi-eht-results.txt', header=None, 
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
    
    plt.figure()#dpi=150)
    th_box = plt.boxplot(th_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                      showfliers=True,
                      medianprops=dict(color="red"))
    th_box_label = mpatches.Patch(facecolor='white', edgecolor='black', 
                          label='WiFi 7 MLO-STR (ns-3 Simulation)')
    th_analysis = plt.plot(n_obss_values, [t/1e6 for t in avg_results['total_throughput']], 'go-', label='WiFi 7 MLO-STR (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Throughput (Mbps)', fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 150)
    plt.legend(handles=[th_box_label, th_analysis[0]], loc='best')
    plt.show()

    # Plot reliability (Simulation vs Analysis)
    rel_data = [wifi_eht[wifi_eht['numBSS'] == val]['PacketDropReliability'].values for val in sorted(wifi_eht['numBSS'].unique())]
    positions = sorted(wifi_eht['numBSS'].unique())

    plt.figure()#dpi=150)
    rel_box = plt.boxplot(rel_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                      showfliers=True,
                      medianprops=dict(color="red"))
    rel_box_label = mpatches.Patch(facecolor='white', edgecolor='black', 
                          label='WiFi 7 MLO-STR (ns-3 Simulation)')
    rel_analysis = plt.plot(n_obss_values, [r*100 for r in avg_results['avg_reliability']], 'go-', label='WiFi 7 MLO-STR (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Reliability (%)', fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(50,105)
    plt.legend(handles=[rel_box_label, rel_analysis[0]], loc='best')
    plt.show()

    # Plot delay (Simulation vs Analysis)
    delay_data = [wifi_eht[wifi_eht['numBSS'] == val]['ChannelAccessDelay'].values for val in sorted(wifi_eht['numBSS'].unique())]
    positions = sorted(wifi_eht['numBSS'].unique())

    plt.figure()#dpi=150)
    delay_box = plt.boxplot(delay_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                      showfliers=True,
                      medianprops=dict(color="red"))
    delay_box_label = mpatches.Patch(facecolor='white', edgecolor='black', 
                          label='WiFi 7 MLO-STR (ns-3 Simulation)')
    delay_analysis = plt.plot(n_obss_values, [d*1e3 for d in avg_results['avg_delay']], 'go-', label='WiFi 7 MLO-STR (Analysis)')
    
    plt.xlabel('Number of OBSS', fontsize=12)
    plt.ylabel('Delay (ms)', fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 5)
    plt.legend(handles=[delay_box_label, delay_analysis[0]], loc='best')
    plt.show()

    # Plot performance metrics in different scenarios
    plot_throughput(n_obss_values,result_summary)
    plot_reliability(n_obss_values,result_summary)
    plot_delay(n_obss_values,result_summary)

if __name__ == "__main__":
    main()