import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt

@dataclass
class BandParameters:
    name: str
    frequency: float  # in GHz
    difs: float       # in seconds
    sifs: float       # in seconds
    n_obss: int       # number of OBSS
    p_error: float    # frame error probability

def calculate_collision_probability(tau, n_obss):
    """Calculate the conditional collision probability p."""
    return 1 - np.power(1 - tau, n_obss - 1)

def calculate_failure_probability(tau, n_obss, p_error):
    """Calculate the total failed transmission probability p."""
    p_coll = calculate_collision_probability(tau, n_obss)
    return 1 - (1 - p_coll)*(1 - p_error) 

def calculate_tau(p, w_0, m):
    """Calculate the transmission probability tau."""
    numerator = (1 - np.power(2*p, m) * (1 - p) + 
                np.power(2, m) * (np.power(p, m) - np.power(p, m+1)) * (1 - 2*p))
    denominator = (1 - 2*p) * (1 - np.power(p, m+1))

    if abs(denominator) < 1e-10:
        return 0.5

    return 2 / (w_0 * (numerator / denominator) + 1)

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
    numerator = p_transmit * p_success * (1 - p_error)* packet_length * data_rate * 1/n_obss
    denominator = ((1 - p_transmit) * slot_size + 
                  p_transmit * p_success * (1 - p_error) * t_success + 
                  p_transmit * (1 - p_success) * t_collision + p_transmit*p_success*p_error*t_error)
    return numerator / denominator

def calculate_first_transmission_reliability(tau, p, n_obss):
    """Calculate the reliability."""
    return 0

def calculate_reliability(p, m):
    """Calculate the reliability."""
    return 1 - np.power(p, m + 1)

def calculate_delay(p, p_collision, p_error, m, w_0, slot_size, t_success, t_collision, t_error):
    """Calculate the average delay."""
    p = np.clip(p, 0.0001, 0.9999)
    t_backoff = 0
    
    for i in range(m):
        p_s = (1 - p) * np.power(p, i)
        mean_t_slot = 0
        for j in range(i + 1):
            w = (np.power(2, j) * w_0 - 1) / 2
            mean_t_slot += w * slot_size
        t_backoff += p_s * (t_success + i * (p_collision*t_collision + (1-p_collision)*p_error*t_error)/p + mean_t_slot)
    
    return t_backoff / (1 - np.power(p, m + 1))

def run_simulation_for_band(band_params, common_params):
    """Run the simulation for a specific frequency band."""
    # Unpack common parameters
    alpha = common_params['alpha']
    eps = common_params['eps']
    max_iter = common_params['max_iter']
    tau = common_params['tau']
    w_0 = common_params['w_0']
    m = common_params['m']
    data_rate = common_params['data_rate']
    lambda_poisson = common_params['lambda_poisson']
    phy_header = common_params['phy_header']
    mac_header_size = common_params['mac_header_size']
    ack_size = common_params['ack_size']
    slot_size = common_params['slot_size']
    prop_delay = common_params['prop_delay']
    packet_size = common_params['packet_size']
    
    # Calculate derived parameters
    mac_header = mac_header_size * 8 / data_rate
    ack = ack_size * 8 / data_rate
    packet_length = packet_size * 8 / data_rate
    
    # Iterative calculation
    for it in range(max_iter):
        p = calculate_failure_probability(tau, band_params.n_obss, band_params.p_error)
        tau_new = calculate_tau(p, w_0, m)
        tau = update_value(alpha, tau, tau_new)
        
        if abs(tau_new - tau) < eps:
            print('Last iteration ', it)
            print("Transmission probability (τ): ", tau)
            print("Collision probability (p): ", p)
            break
    
    # Account the channel error probability


    # Calculate performance metrics
    p_collision = calculate_collision_probability(tau, band_params.n_obss)
    p_transmit = calculate_transmission_probability(tau, band_params.n_obss)
    p_success = calculate_success_probability(tau, band_params.n_obss)
    t_success = calculate_success_time(phy_header, mac_header, packet_length, 
                                      band_params.difs, band_params.sifs, ack, prop_delay)
    t_collision = calculate_collision_time(phy_header, mac_header, packet_length, 
                                         band_params.difs, prop_delay)
    t_error = t_success
    
    #stable_data_rate = min(packet_size * 8 * lambda_poisson,data_rate) #No need to use stable data rate in saturated case
    throughput = calculate_throughput(band_params.n_obss, p_transmit, p_success, band_params.p_error, packet_length, data_rate, slot_size, t_success, t_collision, t_error)
    reliability = calculate_reliability(p, m)
    delay = calculate_delay(p, p_collision, band_params.p_error, m, w_0, slot_size, t_success, t_collision, t_error)
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
    print("Debugging delay_sum : ", delay_sum)
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

    
    plt.xlabel('Number of OBSS (n_obss)')
    plt.ylabel('Throughput (Mbps)')
    plt.title('Throughput vs Number of OBSS')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_reliability(n_obss_values, result_summary):
    """Plot reliability against n_obss values."""
    
    plt.figure(figsize=(8, 5))
    
    plt.plot(n_obss_values, [r*100 for r in result_summary['avg_reliability']], 'bo-', label='WiFi 8 JT Th Mode')
    plt.plot(n_obss_values, [r*100 for r in result_summary['jt_reliability']], 'go-', label='WiFi 8 JT Rel Mode')
    plt.plot(n_obss_values, [r*100 for r in result_summary['avg_reliability']], 'r+--', label='WiFi 7 MLO-STR')

    
    plt.xlabel('Number of OBSS (n_obss)')
    plt.ylabel('Reliability (%)')
    plt.title('Packets Received Reliability vs Number of OBSS')
    plt.legend()
    plt.grid(True)
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
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_delay(n_obss_values, result_summary):
    """Plot delay against n_obss values."""
    
    plt.figure(figsize=(8, 5))
    
    plt.plot(n_obss_values, [d*1e3 for d in result_summary['avg_delay']], 'bo-', label='WiFi 8 JT Th Mode')
    plt.plot(n_obss_values, [d*1e3 for d in result_summary['min_delay']], 'go-', label='WiFi 8 JT Rel Mode')
    plt.plot(n_obss_values, [d*1e3 for d in result_summary['avg_delay']], 'r+--', label='WiFi 7 MLO-STR')

    
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
        'eps': 1e-10,
        'max_iter': 10000,
        'tau': 0.2,
        'w_0': 16,
        'm': 6,
        'lambda_poisson' : 1000,
        'data_rate': 121.875e6,
        'phy_header': 48e-6,
        'mac_header_size': 26,
        'ack_size': 14,
        'slot_size': 9e-6,
        'prop_delay': 67e-9,
        'packet_size': 1500
    }
    
    # Define parameters for different bands
    bands = [
        BandParameters(name="2.4 GHz", frequency=2.4, difs=37e-6, sifs=10e-6, n_obss=4, p_error=0.0),
        BandParameters(name="5 GHz", frequency=5.0, difs=43e-6, sifs=16e-6, n_obss=4, p_error=0.0),
        BandParameters(name="6 GHz", frequency=6.0, difs=43e-6, sifs=16e-6, n_obss=4, p_error=0.0)
    ]
    
    # # Run simulation for each band
    # results = [run_simulation_for_band(band, common_params) for band in bands]
    
    # # Display results in a formatted table
    # print("\nSIMULATION RESULTS FOR DIFFERENT FREQUENCY BANDS")
    # print("="*90)
    
    # for result in results:
    #     print(f"\n{result['band']} Band ({result['frequency']} GHz) - Detailed Results:")
    #     print("-"*90)
        
    #     # Display parameters
    #     print("PARAMETERS:")
    #     print(f"• DIFS: {result['difs']*1e6:.2f} μs") 
    #     print(f"• SIFS: {result['sifs']*1e6:.2f} μs")
    #     print(f"• Number of OBSS: {result.get('n_obss', 'N/A')}")
    #     print(f"• Minimum CW size (W₀): {common_params['w_0']}")
    #     print(f"• Max retry limit (m): {common_params['m']}")
        
    #     # Display probabilities
    #     print("\nPROBABILITIES:")
    #     print(f"• Transmission probability (τ): {result['tau']:.6f}")
    #     print(f"• Collision and frame error probability (p): {result['p']:.6f}")
    #     print(f"• Transmission probability (p_tr): {result['p_transmit']:.6f}")
    #     print(f"• Success probability (p_s): {result['p_success']:.6f}")
        
    #     # Display timings
    #     print("\nTIMINGS:")
    #     print(f"• Successful transmission time: {result['t_success']*1e3:.6f} ms")
    #     print(f"• Collision time: {result['t_collision']*1e3:.6f} ms")
        
    #     # Display performance metrics
    #     print("\nPERFORMANCE METRICS:")
    #     print(f"• Throughput: {result['throughput']/1e6:.3f} Mbps")
    #     print(f"• Reliability: {result['reliability']*100:.2f}%")
    #     print(f"• Average delay: {result['delay']*1e3:.3f} ms")
    #     print("-"*90)
    
    # # Additional summary table for quick comparison
    # print("\nSUMMARY COMPARISON ACROSS BANDS")
    # print("="*150)
    # print(f"{'Band':<10} | {'τ':<8} | {'p':<8} | {'p_tr':<8} | {'p_s':<8} | {'Throughput':<15} | {'Reliability':<12} | {'Delay':<10} | {'t_succ':<12} | {'t_coll':<12}")
    # print("-"*150)
    
    # for result in results:
    #     print(f"{result['band']:<10} | {result['tau']:.6f} | {result['p']:.6f} | "
    #           f"{result['p_transmit']:.6f} | {result['p_success']:.6f} | "
    #           f"{result['throughput']/1e6:>10.3f} Mbps | "
    #           f"{result['reliability']*100:>11.2f}% | "
    #           f"{result['delay']*1e3:>7.3f} ms | "
    #           f"{result['t_success']*1e3:>9.3f} ms | "
    #           f"{result['t_collision']*1e3:>10.3f} ms")

    # final = calculate_final_metrics(results)

    # print("\nFINAL PERFORMANCE METRICS")
    # print("="*60)
    # print(f"• Total throughput (Th-mode): {final['total_throughput']/1e6:.3f} Mbps")
    # print(f"• Maximum throughput (Rel-mode): {final['max_throughput']/1e6:.3f} Mbps")
    # print(f"• Average reliability (Th-mode): {final['avg_reliability']*100:.2f}%")
    # print(f"• Reliability (Rel-mode): {final['jt_reliability']*100:.2f}%")
    # print(f"• Average  (Th-mode): {final['avg_delay']*1e3:.3f} ms")
    # print(f"• Minimum delay (Rel-mode): {final['min_delay']*1e3:.3f} ms")

    n_obss_values = list(range(1, 5))
    result_summary = run_analysis_for_n_obss(n_obss_values, bands, common_params)

    plot_throughput(n_obss_values,result_summary)
    plot_reliability(n_obss_values,result_summary)
    plot_first_trans_reliability(n_obss_values,result_summary)
    plot_delay(n_obss_values,result_summary)
    
if __name__ == "__main__":
    main()
