import numpy as np
import math

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
    mean_retransmission = (p*(1-np.power(p,m))/(1-p))
    mean_retransmission_time = (mean_backoff_time*slot_size)/(1-delta)+(p_collision*t_collision+(1-p_collision)*p_error*t_error)/p
    mean_transmission_time = (mean_backoff_time*slot_size)/(1-delta)+t_success

    mean_service_time = mean_retransmission*mean_retransmission_time + mean_transmission_time
    return mean_service_time

def calculate_mean_backoff_slot(p, m, w_0):
    mean_total_slot = 0
    for i in range(m):
        p_s = (1 - p) * np.power(p, i)
        mean_slot = 0
        for j in range(i + 1):
            w = (np.power(2, j) * w_0 - 1) / 2
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

def calculate_throughput(p_transmit, p_success, p_error, packet_length, data_rate, slot_size, t_success, t_collision, t_error):
    """Calculate the throughput."""
    numerator = p_transmit * p_success * (1 - p_error)* packet_length * data_rate
    denominator = ((1 - p_transmit) * slot_size + 
                  p_transmit * p_success * (1 - p_error) * t_success + 
                  p_transmit * (1 - p_success) * t_collision + p_transmit*p_success*p_error*t_error)
    return numerator / denominator

def calculate_reliability(p, m):
    """Calculate the reliability."""
    return 1 - np.power(p, m + 1)

def calculate_delay(mean_service_time, mean_waiting_time):
    """Calculate the average delay."""
    delay = mean_service_time + mean_waiting_time
    
    return delay

def main():
    # Configuration parameters
    alpha = 0.5  # iteration rate
    eps = 1e-6   # convergence threshold
    max_iter = 50000  # maximum iterations
    
    # Wifi environment parameters
    tau = 0.5  # initial transmission probability
    rho = 0.5  # intial utilization factor
    n_obss = 3  # number of OBSS
    w_0 = 16    # minimum contention window size
    m = 6       # maximum retry limit
    n_links = 4 # number of links in MLO
    
    # Physical layer parameters
    lambda_poisson = 300 # packet arrival rate in poisson distribution
    p_error = 0.0         # frame error probability in channel
    data_rate = 7.3125e6  # Data rate in bps
    phy_header = 52e-6    # PHY header duration in seconds
    mac_header_size = 26  # MAC header size in bytes
    packet_size = 700     # Packet size in bytes
    mac_header = mac_header_size * 8 / data_rate
    ack_size = 14         # ACK size in bytes
    ack = ack_size * 8 / data_rate
    slot_size = 9e-6      # Slot duration in seconds
    prop_delay = 3e-9     # Propagation delay for 1m distance
    difs = 43e-6          # DIFS duration
    sifs = 16e-6          # SIFS duration
    packet_length = packet_size * 8 / data_rate
    
    t_success = calculate_success_time(phy_header, mac_header, packet_length, difs, sifs, ack, prop_delay)
    t_collision = calculate_collision_time(phy_header, mac_header, packet_length, difs, prop_delay)
    t_error = t_success

    # Iterative calculation
    for it in range(max_iter):
        p_collision = calculate_collision_probability(tau, n_obss)
        p_transmit = calculate_transmission_probability(tau, n_obss)
        p_success = calculate_success_probability(tau, n_obss)

        p = calculate_failure_probability(tau, n_obss, p_error)
        delta = calculate_delta(p_transmit, p_success, p_error, slot_size, t_success, t_collision, t_error)

        mean_backoff_slot = calculate_mean_backoff_slot(p, m, w_0)
        mean_backoff_time = calculate_mean_backoff_time(rho, n_links, mean_backoff_slot)

        mean_service_time = calculate_mean_service_time(p, p_collision, p_error, m, delta, slot_size, t_collision, t_error, mean_backoff_time, t_success)

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
    p = calculate_failure_probability(tau, n_obss, p_error)
    p_collision = calculate_collision_probability(tau, n_obss)
    p_transmit = calculate_transmission_probability(tau, n_obss)
    p_success = calculate_success_probability(tau, n_obss)
    mean_service_time = calculate_mean_service_time(p, p_collision, p_error, m, delta, slot_size, t_collision, t_error, mean_backoff_time, t_success)
    mu = calculate_mu(mean_service_time)
    mean_waiting_time = calculate_mean_waiting_time(rho, n_links, lambda_total)

    rho = lambda_total/(n_links*mu)

    # Calculate performance metrics
    stable_data_rate = min(packet_size * 8 * lambda_poisson,data_rate)
    throughput = calculate_throughput(p_transmit, p_success, p_error, packet_length, stable_data_rate, slot_size, t_success, t_collision, t_error)
    reliability = calculate_reliability(p, m)
    delay = calculate_delay(mean_service_time, mean_waiting_time)
    
    # Display results
    print("Iterative Solution Results")
    print(f"Final transmission probability (τ): {tau:.6f}")
    print(f"Final conditional collision probability (p): {p:.6f}")
    print(f"Final arrival rate (lambda): {lambda_total:.6f}")
    print(f"Final service rate (mu): {mu:.6f}")
    print(f"Channel occupancy (delta): {delta:.6f}")
    print(f"Final utilization factor (ρ): {rho:.6f}\n") 
    if rho > 1 or rho < 0 :
        print('Warning!!! Queue is not in a steady state')
    
    print("Timings and Probabilities")
    print(f"Probability of transmission (p_tr): {p_transmit:.6f}")
    print(f"Probability of success (p_s): {p_success:.6f}")
    print(f"Mean service time: {mean_service_time*1e3:.3f} ms")
    print(f"Mean waiting time: {mean_waiting_time*1e3:.3f} ms\n")
    
    print("Performance Metrics")
    print(f"Throughput: {throughput/1e6:.3f} Mbps")
    print(f"Reliability: {reliability*100:.2f}%")
    print(f"Average delay: {delay*1e3:.3f} ms")

if __name__ == "__main__":
    main()