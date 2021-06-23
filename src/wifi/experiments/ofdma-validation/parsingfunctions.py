import numpy as np
from io import StringIO
import itertools
import xarray
import pandas as pd
import seaborn as sns
sns.set_style("whitegrid")
import copy
import sem
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import cm
import ast

#######################
# Per-Flow Statistics #
#######################

def parse_per_flow_statistics(result):
    flowFiles = list(filter(lambda item: "flowStats" in item[0], result['output'].items()))
    flow_counter = 1
    flows = []
    for flowFilename, flowContents in flowFiles:
        flowData = flowContents.splitlines()[0].split(" ")
        if len(flowContents.splitlines()) > 1:
            flowLatencies = flowContents.splitlines()[1].split(" ")[:-1]
        else:
            flowLatencies = []
        if len(flowLatencies) <= 1:
            flowLatencies = []
        flow = {
            "flowId": flow_counter,
            "bss": int(flowData[0]),
            "staId": int(flowData[1]),
            "direction": flowData[2],
            "ac": flowData[3],
            "transport": flowData[4],
            "throughput": float(flowData[5]),
            "expectedDataRate": float(flowData[6]),
            "actualDataRate": float(flowData[7]),
            "packetLossRate": float(flowData[8]),
            "droppedPackets": int(flowData[9]),
            "txBytes": int(flowData[10]),
            "txPackets": int(flowData[11]),
            "rxBytes": int(flowData[12]),
            "rxPackets": int(flowData[13]),
            "latencySamples": [float(i) for i in flowLatencies]
            }
        flows.append(flow)
        flow_counter += 1
    return flows

def get_flow_info(result):
    flows = parse_per_flow_statistics(result)
    if (len(flows) == 0):
        print(sem.utils.get_command_from_result('wifi-multi-ap', result))
        return [float('nan') for i in range(30)]
    return [list(f.values())[:-1] for f in flows]

def get_flow_latencies(result):
    flows = parse_per_flow_statistics(result)
    return [[flow['flowId'], flow['bss'], flow['ac'], flow['direction'], flow['transport'], item] for flow in flows for item in flow['latencySamples']]

def parse_pairwise_per_ac_statistics(result):
    parsed_statistics = []
    for f in result['output'].keys():
        if "pairwise" in f:
            contents = result['output'][f]
            expired, rejected, failed = [float(i) for i in contents.splitlines()[0].split(" ")]
            l2Latency = [float(i) for i in contents.splitlines()[1].split(" ")[:-1]]
            pairwiseHol = [float(i) for i in contents.splitlines()[2].split(" ")[:-1]]
            ampduSize = [float(i) for i in contents.splitlines()[3].split(" ")[:-1]]
            ampduRatio = [float(i) for i in contents.splitlines()[4].split(" ")[:-1]]
            stats = {
                'bss': int(f.split("-")[1]),
                'direction': str.upper(f.split("-")[2]),
                'staId': int(f.split("-")[3]),
                'ac': str.upper(f.split("-")[4]),
                'expired': [expired],
                'rejected': [rejected],
                'failed': [failed],
                'l2Latency': l2Latency,
                'pairwiseHol': pairwiseHol,
                'ampduSize': ampduSize,
                'ampduRatio': ampduRatio
                }
            parsed_statistics += [stats]
    return parsed_statistics

def get_pairwise_samples(result, quantity):
    parsed_statistics = parse_pairwise_per_ac_statistics(result)
    return [[stat['bss'], stat['direction'], stat['staId'], stat['ac'], item] for stat in parsed_statistics for item in stat[quantity]]

def parse_per_ac_statistics(result):
    parsed_statistics = []
    for f in result['output'].keys():
        if "perac" in f:
            contents = result['output'][f]
            droppedByQueueDisc = [float(i) for i in contents.splitlines()[0].split(" ")]
            txopDuration = [float(i) for i in contents.splitlines()[1].split(" ")[:-1]]
            queueDiscSojournTime = [float(i) for i in contents.splitlines()[2].split(" ")[:-1]]
            aggregateHol = [float(i) for i in contents.splitlines()[3].split(" ")[:-1]]
            stats = {
                'bss': int(f.split("-")[1]),
                'staId': int(f.split("-")[2]),
                'ac': str.upper(f.split("-")[3]),
                'droppedByQueueDisc': [droppedByQueueDisc],
                'txopDuration': txopDuration,
                'queueDiscSojournTime': queueDiscSojournTime,
                'aggregateHol': aggregateHol,
                }
            parsed_statistics += [stats]
    return parsed_statistics

def get_per_ac_samples(result, quantity):
    parsed_statistics = parse_per_ac_statistics(result)
    return [[stat['bss'], stat['staId'], stat['ac'], item] for stat in parsed_statistics for item in stat[quantity]]

def parse_per_bss_statistics(result, stat_name):
    stat = []
    for f in result['output'].keys():
        if stat_name in f:
            contents = result['output'][f]
            bss = int(f.split("-")[-1])
            if len(contents.splitlines()) > 0:
                stat.append([bss, [float(i) for i in contents.splitlines()[0].split(" ")[:-1]]])
    return stat

def get_aggregate_samples(result, quantity):
    parsed_statistics = parse_per_bss_statistics(result, quantity)
    return [[bss[0], item] for bss in parsed_statistics for item in bss[1]]

def get_phy_states(result):
    output = []
    for line in result['output']['WifiPhyStateLog.txt'].splitlines():
        cur_time, context, time, duration, state = line.split(" ")
        if state == "TX":
            output += [[int(context), int(time), int(duration), state]]
    return output

# Other plotting functions
##########################

def plot_phy_state (result, filename=None):
    plotstatemap = {
        "RX": False,
        "TX": True,
        "IDLE": False,
        "CCA_BUSY": False,
        "SWITCHING": False,
        "SLEEP": False,
        "OFF": False,
    }
    statecolormap = {
        "RX": 'r',
        "TX": 'b',
        "IDLE": 'grey',
        "CCA_BUSY": 'grey',
        "SWITCHING": 'grey',
        "SLEEP": 'grey',
        "OFF": 'grey',
    }
    preambletextmap = {
        "HE_TB": 'TB',
        "HE_SU": 'SU',
        "HE_MU": 'MU',
        "LONG": '',
    }
    statetextmap = {
        "RX": '',
        "TX": '',
        "IDLE": '',
        "CCA_BUSY": '',
        "SWITCHING": '',
        "SLEEP": '',
        "OFF": '',
    }
    psdutextmap = {
        'CTL_ACK': "A",
        'CTL_BACKRESP': "BA",
        'CTL_BACKREQ': "BACKREQ",
        'QOSDATA_NULL': "QoSNull",
        'QOSDATA': "",
        'CTL_TRIGGER': "TF",
        'MGT_BEACON': "Beacon",
        'MGT_ASSOCIATION_REQUEST': "ARq",
        'MGT_ASSOCIATION_RESPONSE': "ARe",
        'MGT_ACTION': "MA",
        "ANN": "ANN",
        "AQR": "AQR",
        "AQRP": "AQRP",
    }
    for line in result['output']['WifiPhyStateLog.txt'].splitlines():
        cur_time, context, time, duration, state = line.split(" ")
        cur_time = float(cur_time)
        context = float(context)
        time = float(time)
        duration = float(duration)
        if plotstatemap[state]:
            plt.plot([time, time+duration], [context, context],
                        statecolormap[state], linewidth=2)
            if statetextmap.get(state):
                plt.text(time+duration/2, context + 0.05,
                            statetextmap[state],
                            ha='center', fontsize=10, clip_on=True)
    allmpdutypes = set()
    allpreambles = set()
    for line in result['output']['PsduForwardedDown.txt'].splitlines():
        cur_time, context, preamble, *mpdu_types = line.strip().split(" ")
        cur_time = float(cur_time)
        context = float(context)
        contextoffset = 0
        allpreambles.add(preamble)
        plt.text(cur_time, context - 0.4,
                    preambletextmap.get(preamble, None),
                    ha='left', fontsize=10, clip_on=True)
        plt.scatter(cur_time, context, color='b', marker='x')
        for mpdu_type in mpdu_types:
            allmpdutypes.add(mpdu_type)
            if psdutextmap.get(mpdu_type):
                plt.text(cur_time, context - 0.6 - contextoffset,
                            psdutextmap[mpdu_type],
                            ha='left', fontsize=10, clip_on=True)
                contextoffset += 0.15
    # Plot TXOPs
    for line in result['output']['txops.txt'].splitlines():
        bss, nodeId, context, time, duration = line.strip().split(" ")
        bss = int(bss)
        nodeId = int(nodeId)
        context = int(context)
        time = int(time)
        duration = int(duration)
        plt.plot ([time, time+duration],
                    [context, context],
                    'grey',
                    linewidth=20,
                    solid_capstyle='butt',
                    alpha=0.3)
    print(allmpdutypes)
    print(allpreambles)
    nDevicesPerBss = result['params']['nEhtStations'] + result['params']['nHeStations'] + 1
    nBsss = result['params']['nBss']
    apCount = 1
    staCount = 1
    for sta in range(int(nDevicesPerBss * nBsss)):
        staType = "AP"+str(apCount) if sta % nDevicesPerBss == 0 else "STA"+str(staCount)
        if sta % nDevicesPerBss == 0:
            apCount += 1
            staCount = 1
        else:
            staCount += 1
        plt.text(0.3*duration, sta, staType, fontsize=12, va='center')
    plt.title("Visualization of status of devices")
    plt.xlabel("Time [ns]")
    plt.ylim(bottom=-1, top=nDevicesPerBss*nBsss+1)
    plt.gca().spines['left'].set_visible(False)
    plt.yticks (list(range(15)), labels=["" for _ in range(15)])
    plt.grid(axis='x', which='both')
    if filename:
        plt.savefig(filename, bbox_inches='tight')
        plt.close()

def plot_device_locations (result, filename=None):
    colors=list(itertools.islice(iter(cm.rainbow(np.linspace(0,1,result['params']['nBss']))), result['params']['nBss']))
    plt.figure()
    for line in result['output']['locations.txt'].splitlines():
        bss, dev, x, y = line.split(" ")
        plt.text(float(x), float(y), dev, color=colors[int(bss)])
    plt.ylim(-20, 20)
    plt.xlim(-20, 20)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Simulated network topology")
    if filename:
        plt.savefig(filename, bbox_inches='tight')
        plt.close()
    else:
        plt.show()

def plot_cw_and_bo (result):
    bo = pd.DataFrame(data=[[int(i) for i in entry.split(" ")] for entry in result['output']['BackoffLog.txt'].splitlines()],
                      columns=['time', 'device', 'value'])
    cw = pd.DataFrame(data=[[int(i) for i in entry.split(" ")] for entry in result['output']['CWLog.txt'].splitlines()],
                      columns=['time', 'device', 'oldCw', 'value'])
    cw = cw.drop(columns='oldCw')
    cw = cw.assign(quantity='cw')
    bo = bo.assign(quantity='bo')
    cwandbo = pd.concat([cw, bo], ignore_index=True)
    sns.relplot(data=cwandbo,
                x='time',
                y='value',
                row='device',
                style='quantity',
                kind='line')
