##############
# Simulation #
##############
import numpy as np
import copy
import sem
import matplotlib.pyplot as plt

def adapt_dictionary_for_sims(params, step=4, simulationTime=1, dataRate=100):
    validation_params_sim = copy.deepcopy(params)
    validation_params_sim['dlFlowDataRate'] = [dataRate]
    validation_params_sim['ulFlowDataRate'] = [dataRate]
    validation_params_sim['simulationTime'] = [simulationTime]
    params['nEhtStations'] = [0]
    validation_params_sim['nStations'] = list(range(params['nStations'][0], params['nStations'][-1]+1, step))
    return validation_params_sim

def plot_with_cis(results, runs, x, multiple_lines=False):
    results = results.squeeze()
    std = np.transpose(results.reduce(np.std, 'runs').data)
    avg = np.transpose(results.reduce(np.mean, 'runs').data)
    # ci = 1.96 * std / np.sqrt(runs)
    # ci = 2.32 * std / np.sqrt(runs)
    ci = 2.576 * std / np.sqrt(runs)
    # print(avg)
    # avg.plot.line('-', x=x)
    medianprops = dict(linestyle='None')
    boxprops = dict(linestyle='None')
    whiskerprops = dict(linewidth=1)
    ax = plt.gca()
    if len(avg.shape) > 1:
        for i in range(len(avg)):
            # p = ax.plot(results.coords[x].data, avg[i], ':')
            for x_idx, x_value in enumerate(results.coords[x].data):
                item = {}
                item["med"] = avg[i][x_idx]
                item["q1"] = avg[i][x_idx]
                item["q3"] = avg[i][x_idx]
                item["whislo"] = avg[i][x_idx] - ci[i][x_idx]
                item["whishi"] = avg[i][x_idx] + ci[i][x_idx]
                stats = [item]
                ax.bxp(stats,
                       positions=[x_value],
                       showfliers=False,
                       medianprops=medianprops,
                       boxprops=boxprops,
                       whiskerprops=whiskerprops)
                # ax.fill_between(results.coords[x].data, (avg[i]-ci[i]), (avg[i]+ci[i]), color=p[0].get_color(), alpha=.1)
    else:
        # p = ax.plot(results.coords[x].data, avg, ':')
        for x_idx, x_value in enumerate(results.coords[x].data):
            item = {}
            item["med"] = avg[x_idx]
            item["q1"] = avg[x_idx]
            item["q3"] = avg[x_idx]
            item["whislo"] = avg[x_idx] - ci[x_idx]
            item["whishi"] = avg[x_idx] + ci[x_idx]
            stats = [item]
            ax.bxp(stats,
                   positions=[x_value],
                   showfliers=False,
                   medianprops=medianprops,
                   boxprops=boxprops,
                   whiskerprops=whiskerprops)


def get_simulation_campaign(validation_params, runs, overwrite):
    campaign = sem.CampaignManager.new('../../../..', 'wifi-ofdma-validation',
                                       'ofdma-validation-results',
                                       runner_type='ParallelRunner',
                                       overwrite=overwrite,
                                       optimized=True,
                                       check_repo=False,
                                       max_parallel_processes=8)
    params = copy.deepcopy(validation_params)
    params['verbose'] = [False]
    # params['printToFile'] = [True]
    campaign.run_missing_simulations(params, runs)
    return campaign

def get_metrics(result):
    lines = iter(result['output']['stdout'].splitlines())
    dl = ul = dllegacy = ullegacy = hol = dlmucomp = hetbcomp = 0
    # dls = uls = dlslegacy = ulslegacy = []
    dls = uls = []
    while (True):
        line = next(lines, None)
        if line is None:
            break
        if "Per-AC" in line:
            line = next(lines, None)
            line = next(lines, None)
            dls += [float(line.split(" ")[-1])]
            line = next(lines, None)
            line = next(lines, None)
            uls += [float(line.split(" ")[-1]) if line.split(" ")[-1] else 0]
            # line = next(lines, None)
            # line = next(lines, None)
            # dlslegacy += [float(line.split(" ")[-1])]
            # line = next(lines, None)
            # line = next(lines, None)
            # ulslegacy += [float(line.split(" ")[-1])]
        if "Throughput (Mbps) [DL]" in line:
            # Go down to total
            while (True):
                line = next(lines, None)
                if line is None:
                    break
                if "TOTAL:" in line:
                    dl = float(line.split(" ")[-1])
                    break
        if "Throughput (Mbps) [UL]" in line:
            # Go down to total
            while (True):
                line = next(lines, None)
                if line is None:
                    break
                if "TOTAL:" in line:
                    ul = float(line.split(" ")[-1])
                    break
        if "Throughput (Mbps) [DL] LEGACY" in line:
            # Go down to total
            while (True):
                line = next(lines, None)
                if line is None:
                    break
                if "TOTAL:" in line:
                    dllegacy = float(line.split(" ")[-1])
                    break
        if "Throughput (Mbps) [UL] LEGACY" in line:
            # Go down to total
            while (True):
                line = next(lines, None)
                if line is None:
                    break
                if "TOTAL:" in line:
                    ullegacy = float(line.split(" ")[-1])
                    break
        if "Pairwise Head-of-Line delay" in line:
            # Go down to total
            while (True):
                line = next(lines, None)
                if line is None:
                    break
                if "TOTAL:" in line:
                    count = float(line.split("[")[1].split("]")[0])
                    if (count == 0):
                        hol = 0
                        break
                    hol = float(line.split("<")[1].split(">")[0])
                    break
        if "DL MU PPDU completeness" in line:
            # Go down to total
            line = next(lines, None)
            line = next(lines, None)
            dlmucomp = 0
            break
        if "HE TB PPDU completeness" in line:
            # Go down to total
            line = next(next(lines, None))
            line = next(lines, None)
            hetbcomp = 0
            break
    return [dl, ul, dllegacy, ullegacy, hol, dlmucomp, hetbcomp]

def print_detailed_simulation_output (validation_params, overwrite=False):

    campaign = sem.CampaignManager.new('../../../../', 'wifi-ofdma-validation',
                                       'validation-results',
                                       runner_type='ParallelRunner',
                                       overwrite=overwrite,
                                       check_repo=False)
    params = copy.deepcopy(validation_params)
    params['verbose'] = [True]
    # params['printToFile'] = [True]
    campaign.run_missing_simulations(params)

    example_result = next(campaign.db.get_complete_results(params))

    print(example_result['output']['stdout'])

    print(sem.utils.get_command_from_result('wifi-ofdma-validation',
                                            example_result))

    for line in example_result['output']['stdout'].splitlines():
        if "Starting statistics" in line:
            start = int(line.split(" ")[-1])
        if "Stopping statistics" in line:
            stop = int(line.split(" ")[-1])

    print("Transmission times for devices: ")
    for line in example_result['output']['WifiPhyStateLog.txt'].splitlines():
        cur_time, context, time, duration, state = line.split(" ")
        if state == "TX" and float(time) > start and float(time) < stop:
            print("Time %s: device %s transmitted for %s" % (time, context, duration))

    print("Simulation throughput: %s" % get_metrics(example_result))
    return example_result


def get_simulation_metrics(validation_params, runs=10, overwrite=False, results_folder='validation-results', verbose=False):

    campaign = sem.CampaignManager.new('../../../../', 'wifi-ofdma-validation',
                                       results_folder,
                                       runner_type='ParallelRunner',
                                       overwrite=overwrite,
                                       check_repo=False,
                                       max_parallel_processes=8)

    params = copy.deepcopy(validation_params)
    params['verbose'] = [False]
    if runs == None:
        campaign.run_missing_simulations(params)
    else:
        campaign.run_missing_simulations(params, runs)

    if verbose:
        for result in campaign.db.get_results(params):
            print(sem.utils.get_command_from_result('wifi-ofdma-validation', result))
    throughput_results = campaign.get_results_as_xarray(
        params, get_metrics, ['dl', 'ul', 'dllegacy', 'ullegacy', 'hol',
                              'dlmucomp', 'hetbcomp'], runs).squeeze()

    return throughput_results
