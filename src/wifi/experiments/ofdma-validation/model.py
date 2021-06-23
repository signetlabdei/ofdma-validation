#########
# Model #
#########
import xarray as xr
import pandas as pd
from tqdm import tqdm
from multiprocessing import Pool
import copy
import math
from itertools import product
import numpy as np
import scipy.optimize  # Used to solve the fixed-point problem
import scipy
from functools import reduce
import operator
import random

def dict_product(d):
    keys = d.keys()
    for element in product(*d.values()):
        yield dict(zip(keys, element))

def get_ru_number_bellalta(channel_width, N):
    """
    Map linking the number of users to the width of the assigned RUs, according
    to the model policy. This function tries to fit as many users in each
    transmission opportunity.
    """
    if channel_width == 20e6 and N >= 9:
            return 9
    elif channel_width == 40e6 and N >= 18:
            return 18
    elif channel_width == 80e6 and N >= 37:
            return 37
    return N


def get_ru_number_roundrobin(channel_width, N):
    """
    Map linking the number of users to the width of the assigned RUs, according
    to the Round Robin policy. This function tries to assign the largest size
    of RUs such that all users will be served within two DL TX opportunities.
    """
    if channel_width == 20e6:
        if (N >= 9):
            return 9
        elif (N >= 4):
            return 4
        elif (N >= 2):
            return 2
        elif (N >= 1):
            return 1
    elif channel_width == 40e6:
        if (N >= 18):
            return 18
        elif (N >= 8):
            return 8
        elif (N >= 4):
            return 4
        elif (N >= 2):
            return 2
        elif (N >= 1):
            return 1
    elif channel_width == 80e6:
        if (N >= 37):
            return 37
        elif (N >= 16):
            return 16
        elif (N >= 8):
            return 8
        elif (N >= 4):
            return 4
        elif (N >= 2):
            return 2
        elif (N >= 1):
            return 1
    elif channel_width == 160e6:
        if (N >= 74):
            return 74
        elif (N >= 32):
            return 32
        elif (N >= 16):
            return 16
        elif (N >= 8):
            return 8
        elif (N >= 4):
            return 4
        elif (N >= 2):
            return 2
        elif (N >= 1):
            return 1
    if (N == 0):
        return 1
    raise Exception("Unrecognized channel + N combination")


def get_data_subcarriers(channel_width, N_ru):
    """
    Map linking RU width with number of data subcarriers.
    """
    if channel_width == 20e6:
        if (N_ru <= 1):
            return 234
        elif (N_ru <= 2):
            return 102
        elif (N_ru <= 4):
            return 48
        elif (N_ru <= 9):
            return 24
    elif channel_width == 40e6:
        if (N_ru <= 1):
            return 468
        elif (N_ru <= 2):
            return 234
        elif (N_ru <= 4):
            return 102
        elif (N_ru <= 8):
            return 48
        elif (N_ru <= 18):
            return 24
    elif channel_width == 80e6:
        if (N_ru <= 1):
            return 980
        elif (N_ru <= 2):
            return 468
        elif (N_ru <= 4):
            return 234
        elif (N_ru <= 8):
            return 102
        elif (N_ru <= 16):
            return 48
        elif (N_ru <= 37):
            return 24
    elif channel_width == 160e6:
        if (N_ru <= 1):
            return 2*980
        elif (N_ru <= 2):
            return 980
        elif (N_ru <= 4):
            return 468
        elif (N_ru <= 8):
            return 234
        elif (N_ru <= 16):
            return 102
        elif (N_ru <= 32):
            return 48
        elif (N_ru <= 74):
            return 24


def get_rate_and_coding_for_modulation(mcs):
    """
    Return the bits/symbol and the coding rate associated to a certain MCS.
    """
    bits_per_symbol = [1, 2, 2, 4, 4, 6, 6, 6, 8, 8, 10, 10]
    code_rate = [1/2, 1/2, 3/4, 1/2, 3/4, 2/3, 3/4, 5/6, 3/4, 5/6, 3/4, 5/6]
    return [bits_per_symbol[mcs], code_rate[mcs]]


def compute_model_throughput(params, print_stuff=False):

    if params['mcs'] == 'ideal':
        mcs = 11
    else:
        mcs = params['mcs']
    Y_m, Y_c = get_rate_and_coding_for_modulation(mcs)
    NHe = params['nStations'] # Number of user stations
    L_D = params['frameSize'] * 8 # Frame size in bits
    N_a = params['Na']  # Number of aggregated frames

    if (params['dl'] == 'None'):  # Disable AP traffic
        CW_min_ap = 15e1000  # Minimum contention window
    else:
        CW_min_ap = params['cwMin']  # Minimum contention window

    if params['ul'] == 'None':
        CW_min_he_sta = 15e1000  # Disable STA traffic
    else:
        CW_min_he_sta = params['cwMin']  # STAs contend just like the AP

    # Number of backoff stages
    m_ap = m_he_sta = 6
    # OFDM symbol duration (12.8 symbol + 3.2 GI)
    # sigma = 16e-6
    # OFDM symbol duration (12.8us symbol + 800ns GI)
    sigma = 12.8e-6 + 800e-9
    # OFDM symbol duration (12.8us symbol + 1600ns GI)
    sigma_1600 = 12.8e-6 + 1.6e-6
    # Duration of an empty backoff slot
    T_e = 9e-6
    # Channel width
    B = params['channelWidth']*1e6
    # Function determining how we get the number of Resource Units to employ.
    if params['scheduler'] == 'rr':
        ru_number_function = get_ru_number_roundrobin
    elif params['scheduler'] == 'bellalta':
        ru_number_function = get_ru_number_bellalta
    else:
        raise Exception("Unknown scheduler: %s" % params['scheduler'])
    # Get the number of resource units to employ in MU transmissions
    V_u = max(1, min(NHe, ru_number_function(B, NHe)))
    # Number of data sub-carriers in the bandwidth for a single RU
    Y_sc = get_data_subcarriers(B, ru_number_function(B, NHe))
    # Number of data sub-carriers in the whole channel, for SU transmissions
    Y_sc_su = get_data_subcarriers(B, ru_number_function(B, 1))
    # We assume a single Spatial Stream
    V_s = 1
    # Bits per OFDM symbol in the current setup, in MU transmissions
    r = V_s * Y_m * Y_c * Y_sc
    # Bits per OFDM symbol for SU transmissions
    r_su = V_s * Y_m * Y_c * Y_sc_su
    # Bits/OFDM symbol in 6Mbps legacy mode
    r_legacy_6 = 1 * 48 * 1/2
    # Bits/OFDM symbol in 24Mbps legacy mode
    r_legacy_24 = 4 * 48 * 1/2
    # Legacy OFDM symbol duration
    sigma_legacy = 4e-6
    # Whether to employ MU or SU traffic at the AP
    alpha = 0 if params['dl'] == 'mu' else 1
    beta = 0
    if params['dl'] == 'mu' and (params['ul'] == 'su' or params['ul'] == 'None'):
        beta = 1
    elif params['dl'] == 'mu' and params['ul'] == 'mu':
        beta = 0.5
    # TXOP duration, given in microseconds -> Convert to seconds
    maxTxopDuration = params['maxTxopDuration'] * 1e-6
    # PHY header lengths for different kinds of packets
    T_PHY_HE_TB = 97e-6
    T_PHY_HE_MU = 48e-6
    T_PHY_legacy = 20e-6
    T_PHY_HE_SU = 44e-6
    L_MD = 4*8  # MPDU Delimiter
    L_MH = 26*8  # MAC header length
    L_UDP = 8*36  # UDP header length
    # Length of an MPDU: UDP header + MAC header + APP Payload
    L_MPDU = L_UDP + L_MH + L_D
    # MAC trailer
    L_MT = 4*8
    # See MpduAggregator for padding formula
    L_PAD = ((4 - ((L_MPDU/8) % 4)) % 4) * 8
    # The following is the length if we were to instruct each STA
    # L_MU_BAR_TF = (8 + V_u * 9 + 2) * 8
    # However, since we are aggregating this TF, we only need information about
    # a single user (i.e., the destination STA)
    L_MU_BAR_TF = (8 + V_u * 9 + 2) * 8
    # L_trigger_BASIC is the length of the TF MPDU instructing devices on how
    # to perform simultaneous transmissions.
    # 8 Header + 6 User info * nUsers + 2 Padding
    # 16 as header, 4 as padding
    L_trigger_BASIC = 8 * (16 + (8 + 6 * V_u + 2) + 4)  # In bits
    # Length of the TF sent by the AP to poll for BSRs
    L_trigger_BSRP = 224 + 48 * V_u
    # This is the length of the BSR report as a response to a BSRP
    # There are 8 QOS_NULL frames, with 2 bytes of padding
    # We subtract 2 * 8 since the last aggregated frame has no padding.
    L_BSR = (8 * (L_MD + 30 * 8 + 2 * 8) - 2 * 8)
    # Length of the Multi-User Block ACK, sent in response to a MU DL
    # transmission.
    L_BACK_MU = 56 * 8
    # Length of Single-User ACKs
    L_BACK = 56 * 8
    # Short Inter-Frame Spacing
    T_SIFS = 16e-6
    # Arbitration Inter-Frame Spacing
    T_AIFS = 34e-6
    # Length of Multi-User A-MPDUs. These include MU_BAR_TF, the Trigger Frame
    # used to coordinate the simultaneous UL MU BACK
    # TODO If aggregated, do not add BAR
    L_MU_BAR = L_MD + L_MU_BAR_TF
    # Length of Single-User A-MPDUs, the same as above but without the
    # aggregation of a MU_BAR_TF.
    # Here we subtract L_PAD since there is no padding in the last MPDU

    T_trigger_basic = (T_PHY_legacy + np.ceil((L_trigger_BASIC) / r_legacy_6) *
                       sigma_legacy)
    T_MU_BAR = (T_PHY_legacy + np.ceil((L_MU_BAR) / r_legacy_6) *
                       sigma_legacy)
    T_BACK = (T_PHY_legacy + np.ceil((L_BACK) / r_legacy_24) * sigma_legacy)
    # 16e-6 is the PHY HEADER duration
    T_BSR = (T_PHY_HE_TB + 16e-6 + np.ceil((L_BSR) / r) * sigma)
    # T_BACK_MU = (T_PHY_HE_TB + np.ceil((L_BACK_MU) / r) * sigma)
    T_BACK_MU = 112e-6
    # T_BAR = (T_PHY_HE_MU + 16e-6 + np.ceil(L_MU_BAR / r) * sigma)
    T_BAR = 32e-6
    # T_MU_BAR = 172e-6




    # Computation of the length of a MU DL PPDU
    def get_L_MU_DL_AMPDU(n_a_mu_dl):
        if params['ackSeqType'] == 'ACK-SU-FORMAT':
            return (n_a_mu_dl * (L_MD + L_MPDU + L_MT + L_PAD) - L_PAD)
        elif params['ackSeqType'] == 'MU-BAR':
            return (n_a_mu_dl * (L_MD + L_MPDU + L_MT + L_PAD) - L_PAD)
        elif params['ackSeqType'] == 'AGGR-MU-BAR':
            return (n_a_mu_dl * (L_MD + L_MPDU + L_MT + L_PAD) + (L_MD + L_MU_BAR_TF))

    def get_T_mu_d_D(n_a_mu_dl):
        return T_PHY_HE_MU + 16e-6 + np.ceil(get_L_MU_DL_AMPDU(n_a_mu_dl) / r) * sigma

    def get_T_mu_d(n_a_mu, V_u):
        T_mu_d_D = get_T_mu_d_D(n_a_mu)
        if params['ackSeqType'] == 'ACK-SU-FORMAT':
            return (T_mu_d_D + T_SIFS + T_BACK + (V_u - 1) * (T_SIFS + T_BAR + T_SIFS + T_BACK) + T_AIFS)
        elif params['ackSeqType'] == 'MU-BAR':
            return (T_mu_d_D + T_SIFS + T_MU_BAR + T_SIFS + T_BACK_MU + T_AIFS)
        elif params['ackSeqType'] == 'AGGR-MU-BAR':
            return (T_mu_d_D + T_SIFS + T_BACK_MU + T_AIFS)

    # Build the longest MU PPDU that will fit in the TXOP
    n_a_mu_dl = 1
    if maxTxopDuration != 0:
        while True:
            if (n_a_mu_dl > N_a or get_T_mu_d(n_a_mu_dl, V_u) > maxTxopDuration):
                n_a_mu_dl -= 1
                break
            n_a_mu_dl += 1
    else:
        n_a_mu_dl = N_a
    L_MU_DL_AMPDU = get_L_MU_DL_AMPDU(n_a_mu_dl)
    T_mu_d_D = get_T_mu_d_D(n_a_mu_dl)





    # Computation of the length of a MU DL PPDU
    def get_L_MU_UL_AMPDU(n_a_mu_ul):
        return (n_a_mu_ul * (L_MD + L_MPDU + L_MT + L_PAD) - L_PAD)

    # Computation of the length of a MU DL PPDU
    def get_T_mu_u_D(L_MU_UL_AMPDU):
        L_MU_UL_AMPDU = get_L_MU_UL_AMPDU(n_a_mu_ul)
        return T_PHY_HE_TB + 16e-6 + np.ceil(L_MU_UL_AMPDU / r) * sigma_1600

    def get_T_mu_u(n_a_mu_ul):
        T_mu_u_D = get_T_mu_u_D(get_L_MU_UL_AMPDU(n_a_mu_ul))
        return (T_trigger_basic + T_SIFS + T_mu_u_D + T_SIFS + T_BACK + T_SIFS)

    # Build the longest MU PPDU that will fit in the TXOP
    n_a_mu_ul = 1
    if maxTxopDuration != 0:
        while True:
            if (n_a_mu_ul > N_a or get_T_mu_u(n_a_mu_ul) > maxTxopDuration):
                n_a_mu_ul -= 1
                break
            n_a_mu_ul += 1
    else:
        n_a_mu_ul = N_a
    L_MU_UL_AMPDU = get_L_MU_UL_AMPDU(n_a_mu_ul)
    T_mu_u_D = get_T_mu_u_D(L_MU_UL_AMPDU)







    # Computation of the length of a SU PPDU
    def get_L_SU_AMPDU(n_a_su):
        return n_a_su * (L_MD + L_MPDU + L_MT + L_PAD) - L_PAD

    def get_T_su_d(n_a_su):
        L_SU_AMPDU = get_L_SU_AMPDU(n_a_su)
        return T_PHY_HE_SU + np.ceil(L_SU_AMPDU / r_su) * sigma

    # Build the longest SU PPDU that will fit in the TXOP
    n_a_su = 1
    if maxTxopDuration != 0:
        while True:
            if (n_a_su >= N_a or get_T_su_d(n_a_su) > maxTxopDuration):
                n_a_su -= 1
                break
            n_a_su += 1
        n_a_su -= 1
    else:
        n_a_su = N_a
    L_SU_AMPDU = get_L_SU_AMPDU(n_a_su)
    T_su_D = get_T_su_d(n_a_su)






    #####################
    # Core of the model #
    #####################

    def core_function(x):

        # print(x)

        tau_ap, tau_he_sta, p_c_ap, p_c_he_sta = x

        n_he_sta = NHe

        def E(CW_min, m, p_c):
            return (((1 - p_c - p_c * (2 * p_c) ** m) / (1 - 2 * p_c)) *
                    ((CW_min + 1) / 2) - 1/2)

        p_c_ap = 1 - (1 - tau_he_sta)**n_he_sta

        if n_he_sta > 0:
            p_c_he_sta = 1 - ((1 - tau_ap) *
                              (1 - tau_he_sta)**(n_he_sta-1))
        else:
            p_c_he_sta = 0

        tau_ap = 1 / (E(CW_min_ap, m_ap, p_c_ap) + 1)

        if n_he_sta > 0:
            tau_he_sta = 1 / (E(CW_min_he_sta, m_he_sta, p_c_he_sta) + 1)
        else:
            tau_he_sta = 0

        outcome = [tau_ap, tau_he_sta, p_c_ap, p_c_he_sta]

        return np.array(outcome)

    # Use Fixed Point Iteration to solve the problem
    initial_guess = np.zeros(4)
    tau_ap, tau_he_sta, p_c_ap, p_c_he_sta = scipy.optimize.fixed_point(core_function, initial_guess)

    E_ap = (((1 - p_c_ap - p_c_ap * (2 * p_c_ap) ** m_ap) / (1 - 2 * p_c_ap)) * ((CW_min_ap + 1) / 2) - 1/2)
    E_he_sta = (((1 - p_c_ap - p_c_ap * (2 * p_c_ap) ** m_ap) / (1 - 2 * p_c_ap)) * ((CW_min_ap + 1) / 2) - 1/2)

    ############
    # Formulas #
    ############

    # TODO Convert this
    T_trigger_basic = (T_PHY_legacy + np.ceil((L_trigger_BASIC) / r_legacy_6) *
                       sigma_legacy)
    T_BACK = (T_PHY_legacy + np.ceil((L_BACK) / r_legacy_24) * sigma_legacy)
    # 16e-6 is the PHY HEADER duration
    T_BSR = (T_PHY_HE_TB + 16e-6 + np.ceil((L_BSR) / r) * sigma)

    # Computation of transmission times
    T_su = (T_su_D + T_SIFS + T_BACK + T_SIFS)
    T_mu_d = get_T_mu_d(n_a_mu_dl, V_u)
    T_mu_u = maxTxopDuration

    # This is a collision between two SU transmissions (a STA in UL SU and
    # another STA in UL SU)
    T_c_su = T_su_D + T_SIFS
    # This is a collision between a DL MU transmissions and an UL SU
    # transmission (an AP and a STA)
    T_c_mu_dl = T_mu_d
    T_c_mu_ul_sta = T_mu_u
    T_c_mu_ul_ap = 96e-6 + T_SIFS


    #############################################################
    # Probabilities of having a certain outcome at a given slot #
    #############################################################

    n_he_sta = int(NHe)

    # Probability of picking a HE device for a DL transmission
    p_he_device = NHe / (params['nStations'])

    # Probability the AP initiates a DL SU transmission, and there is no
    # collision.
    a1 = (tau_ap *
          p_he_device *
          alpha *
          (1 - p_c_ap))

    # Probability exactly a single non-backed-off STA transmits, and there is no collision
    a2 = (n_he_sta *
          tau_he_sta *
          (1 - p_c_he_sta))

    # Probability the AP initiates a MU DL transmission, no collision
    a3 = ((1 - alpha) *
          beta *
          tau_ap *
          p_he_device *
          (1 - p_c_ap))

    # Probability the AP initiates a MU UL transmission, no collision
    a4 = ((1 - alpha) *
          (1 - beta) *
          tau_ap *
          p_he_device *
          (1 - p_c_ap))

    # Probability no one tries to transmit in a given slot
    b1 = ((1 - tau_ap) *
          (1 - tau_he_sta)**n_he_sta)

    p_he_tx = (1 - (1 - tau_he_sta)**n_he_sta)
    p_atleast_one_sta = p_he_tx

    # Probability of a collision for a DL SU transmission by the AP
    c1 = (alpha *
          tau_ap *
          p_c_ap)

    # Probability of a collision for a DL MU transmission by the AP
    c2 = ((1 - alpha) *
          beta *
          tau_ap *
          p_c_ap)

    # Probability of a collision for an UL MU transmission by the AP
    c3 = ((1 - alpha) *
          (1 - beta) *
          tau_ap *
          p_c_ap)

    # Probability of a collision between stations
    c4 = 1 - a1 - a2 - a3 - a4 - b1 - c1 - c2 - c3

    ########################
    # UL and DL throughput #
    ########################

    T_a1 = T_su
    T_a2 = T_su
    T_a3 = T_mu_d
    T_a4 = T_mu_u
    T_c1 = T_c_su
    T_c2_sta = T_c_su
    T_c2_ap = T_c_mu_dl
    T_c3_sta = T_c_su
    T_c3_ap = T_c_mu_ul_ap
    T_c4 = T_c_su

    all_cases_sta = (b1 * T_e +
                 a1 * (T_a1 + T_e) +
                 a2 * (T_a2 + T_e) +
                 a3 * (T_a3 + T_e) +
                 a4 * (T_a4 + T_e) +
                 c1 * (T_c1 + T_e) +
                 c2 * (T_c2_sta + T_e) +
                 c3 * (T_c3_sta + T_e) +
                 c4 * (T_c4 + T_e))

    all_cases_ap = (b1 * T_e +
                 a1 * (T_a1 + T_e) +
                 a2 * (T_a2 + T_e) +
                 a3 * (T_a3 + T_e) +
                 a4 * (T_a4 + T_e) +
                 c1 * (T_c1 + T_e) +
                 c2 * (T_c2_ap + T_e) +
                 c3 * (T_c3_ap + T_e) +
                 c4 * (T_c4 + T_e))

    # Throughput for HE devices
    S_d = ((a1 * n_a_su * L_D + a3 * V_u * n_a_mu_dl * L_D) / all_cases_ap)
    S_u = ((a2 * n_a_su * L_D + a4 * V_u * n_a_mu_ul * L_D) / all_cases_sta)

    dl_throughput = S_d / 1e6
    ul_throughput = S_u / 1e6

    ################
    # Head-of-line #
    ################

    if params['dl'] == 'su':
        hol = NHe * (T_su + E_ap * T_e) * 1000
    else:
        hol = NHe / V_u * (T_mu_d + E_ap * T_e) * 1000

    if print_stuff:
        print("\n--- Relevant parameters ---\n")
        print("Transmission_rate: %s Mb/s" % (r/sigma/1e6))
        print("DL_MU_A-MPDU_size: %s bytes" % (L_MU_DL_AMPDU / 8))
        print("UL_MU_A-MPDU_size: %s bytes" % (L_MU_UL_AMPDU / 8))
        print("SU_A-MPDU_size: %s bytes" % (L_SU_AMPDU / 8))
        print("SU_Aggregation: %s" % n_a_su)
        print("DL_MU_Aggregation: %s" % n_a_mu_dl)
        print("UL_MU_Aggregation: %s" % n_a_mu_ul)
        print("alpha: %s" % alpha)
        print("beta: %s" % beta)
        print("V_u: %s" % V_u)

        print("\n--- Time quantities ---\n")
        print("T_su_D: %1.1f us" % (T_su_D * 1e6))
        print("T_mu_d_D: %1.1f us" % (T_mu_d_D * 1e6))
        print("T_mu_u_D: %1.1f us" % (T_mu_u_D * 1e6))
        print("T_trigger_basic: %1.1f us" % (T_trigger_basic * 1e6))
        print("L_trigger_basic: %s bytes" % (L_trigger_BASIC/8))
        print("L_BSR: %s bytes" % (L_BSR/8))
        print("T_BSR: %1.1f us" % (T_BSR * 1e6))
        print("T_BACK: %1.1f us" % (T_BACK * 1e6))
        print("T_BACK_MU: %1.1f us" % (T_BACK_MU * 1e6))
        print("T_BAR: %1.1f us" % (T_BAR * 1e6))
        print("T_MU_BAR: %1.1f us" % (T_MU_BAR * 1e6))

        print("\n--- Exchange durations ---\n")
        print("MU_DL: %1.1f us" % (T_mu_d * 1e6))
        print("MU_UL: %1.1f us" % (T_mu_u * 1e6))

        print("\n--- Channel access probabilities ---\n")
        print("n_he_sta: %s" % n_he_sta)
        print("n_he_sta_mu: %s" % n_he_sta_mu)
        print("tau_ap: %s" % tau_ap)
        print("tau_he_sta: %s" % tau_he_sta)
        print("tau_he_sta_mu: %s" % tau_he_sta_mu)
        print("p_c_ap: %s" % p_c_ap)
        print("p_c_he_sta: %s" % p_c_he_sta)
        print("p_c_he_sta_mu: %s" % p_c_he_sta_mu)
        print("E_ap: %s" % E_ap)
        print("E_he_sta: %s" % E_he_sta)
        print("E_he_sta_mu: %s" % E_he_sta_mu)
        print("Successful_DL_SU_probability_a1: %s" % a1)
        print("Successful_UL_SU_probability_a2: %s" % a2)
        print("Successful_DL_MU_probability_a3: %s" % a3)
        print("Successful_UL_MU_probability_a4: %s" % a4)
        print("Collided_DL_SU_probability_c1: %s" % c1)
        print("Collided_DL_MU_probability_c2: %s" % c2)
        print("Collided_UL_MU_probability_c3: %s" % c3)
        print("Collided_UL_STAs_probability_c4: %s" % c4)
        print("Empty_slot_probability_b1: %s" % b1)
        print("Sum_of_probabilities: %s" % (a1 + a2 + a3 + a4 + a5 + a6 + a7 + b1 + c1 + c2 + c3 + c4))

        print("T_a1: %s" % T_a1)
        print("T_a2: %s" % T_a2)
        print("T_a3: %s" % T_a3)
        print("T_a4: %s" % T_a4)
        print("T_a5: %s" % T_a5)
        print("T_a6: %s" % T_a6)
        print("T_a7: %s" % T_a7)

        print("T_c1: %s" % T_c1)
        print("T_c2_sta: %s" % T_c2_sta)
        print("T_c2_ap: %s" % T_c2_ap)
        print("T_c3_sta: %s" % T_c3_sta)
        print("T_c3_ap: %s" % T_c3_ap)
        print("T_c4: %s" % T_c4)

        print("\n--- Throughput ---\n")
        print("TOT_throughput: %s" % (dl_throughput+ul_throughput))
        print("DL_throughput: %s" % dl_throughput)
        print("UL_throughput: %s" % ul_throughput)
        print("HoL: %s" % hol)

    return [dl_throughput, ul_throughput,
            hol]


def print_detailed_model_output(params):
    compute_model_throughput(list(dict_product(params))[0], True)

def get_model_throughput(validation_params):
    params_with_metrics = copy.deepcopy(validation_params)
    params_with_metrics['metrics'] = ['dl', 'ul', 'hol']
    model_output = xr.DataArray(np.zeros([len(i) for i in
                                            params_with_metrics.values()]),
                                            list(zip(list(params_with_metrics.keys()),
                                                    params_with_metrics.values())))
    with Pool() as pool:
        param_list = list(dict_product(validation_params))
        for param_comb, result in zip(param_list,
                                      pool.imap(compute_model_throughput,
                                                tqdm(param_list,
                                                     total=len(param_list), 
                                                     desc="Running model",
                                                    unit="Parameter combination"))):
            model_output.loc[param_comb] = result
    return model_output
