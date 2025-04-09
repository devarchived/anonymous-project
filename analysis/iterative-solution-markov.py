import numpy as np

def calculate_transmission_probability(tau, n_obss):
    """Calculate the conditional collision probability p."""
    return 1 - np.power(1 - tau, n_obss - 1)

def calculate_tau(p, w_0, m):
    """Calculate the transmission probability tau."""
    numerator = (1 - np.power(2*p, m) * (1 - p) + 
                np.power(2, m) * (np.power(p, m) - np.power(p, m+1)) * (1 - 2*p))
    denominator = (1 - 2*p) * (1 - np.power(p, m+1))
    return 2 / (w_0 * (numerator / denominator) + 1)

def update_value(alpha, old_value, new_value):
    """Update a value using exponential moving average."""
    return alpha * old_value + (1 - alpha) * new_value

def calculate_transmission_probability_total(tau, n_obss):
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

def calculate_throughput(p_transmit, p_success, packet_length, data_rate, slot_size, t_success, t_collision):
    """Calculate the throughput."""
    numerator = p_transmit * p_success * packet_length * data_rate
    denominator = ((1 - p_transmit) * slot_size + 
                  p_transmit * p_success * t_success + 
                  p_transmit * (1 - p_success) * t_collision)
    return numerator / denominator

def calculate_reliability(p, m):
    """Calculate the reliability."""
    return 1 - np.power(p, m + 1)

def calculate_delay(p, m, w_0, slot_size, t_success, t_collision):
    """Calculate the average delay."""
    t_backoff = 0
    for i in range(m):
        p_s = (1 - p) * np.power(p, i)
        mean_t_slot = 0
        for j in range(i + 1):
            w = (np.power(2, j) * w_0 - 1) / 2
            mean_t_slot += w * slot_size
        t_backoff += p_s * (t_success + i * t_collision + mean_t_slot)
    
    return t_backoff / (1 - np.power(p, m + 1))

def main():
    # Configuration parameters
    alpha = 0.5  # iteration rate
    eps = 1e-6   # convergence threshold
    max_iter = 10000  # maximum iterations
    
    # Wifi environment parameters
    tau = 0.5  # initial transmission probability
    n_obss = 1  # number of OBSS
    w_0 = 16    # minimum contention window size
    m = 6       # maximum retry limit
    
    # Physical layer parameters
    lambda_poisson = 3000 # packet arrival rate in poisson distribution
    data_rate = 7.3125e6  # Data rate in bps
    phy_header = 52e-6    # PHY header duration in seconds
    mac_header_size = 26  # MAC header size in bytes
    mac_header = mac_header_size * 8 / data_rate
    ack_size = 14         # ACK size in bytes
    ack = ack_size * 8 / data_rate
    slot_size = 9e-6      # Slot duration in seconds
    prop_delay = 3e-9     # Propagation delay for 1m distance
    difs = 43e-6          # DIFS duration
    sifs = 16e-6          # SIFS duration
    packet_size = 700     # Packet size in bytes
    packet_length = packet_size * 8 / data_rate
    
    # Iterative calculation
    for _ in range(max_iter):
        p = calculate_transmission_probability(tau, n_obss)
        tau_new = calculate_tau(p, w_0, m)
        tau = update_value(alpha, tau, tau_new)
        
        if abs(tau_new - tau) < eps:
            break
    
    # Calculate performance metrics
    p_transmit = calculate_transmission_probability_total(tau, n_obss)
    p_success = calculate_success_probability(tau, n_obss)
    t_success = calculate_success_time(phy_header, mac_header, packet_length, difs, sifs, ack, prop_delay)
    t_collision = calculate_collision_time(phy_header, mac_header, packet_length, difs, prop_delay)
    
    stable_data_rate = min(packet_size * 8 * lambda_poisson,data_rate)
    throughput = calculate_throughput(p_transmit, p_success, packet_length, stable_data_rate, 
                                    slot_size, t_success, t_collision)
    reliability = calculate_reliability(p, m)
    delay = calculate_delay(p, m, w_0, slot_size, t_success, t_collision)
    
    # Display results
    print("Iterative Solution Results")
    print(f"Final transmission probability (τ): {tau:.6f}")
    print(f"Final conditional collision probability (p): {p:.6f}\n")
    
    print("Timings and Probabilities")
    print(f"Probability of transmission (p_tr): {p_transmit:.6f}")
    print(f"Probability of success (p_s): {p_success:.6f}")
    print(f"Successful transmission time: {t_success*1e3:.3f} ms")
    print(f"Collision time: {t_collision*1e3:.3f} ms\n")
    
    print("Performance Metrics")
    print(f"Throughput: {throughput/1e6:.3f} Mbps")
    print(f"Reliability: {reliability*100:.2f}%")
    print(f"Average delay: {delay*1e3:.3f} ms")

if __name__ == "__main__":
    main()