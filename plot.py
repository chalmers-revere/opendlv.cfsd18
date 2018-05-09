import os
import sys
import matplotlib.pyplot as plt
import numpy as np

#path = 'C:/Users/zhuan/PycharmProjects/cfsdplot/data'
#plottype = 2

# 1 is line chart, 2 is stacked bar chart
plottype = sys.argv[1]
path = sys.argv[2]

def seperatefiles(folderpath):
    f_cpu = [];
    f_mem = [];
    for f in os.listdir(folderpath):
        if f.endswith(".cpu"):
            f_cpu.append(f)
        elif f.endswith(".mem"):
            f_mem.append(f)
    return [f_cpu, f_mem]


def plotstackedbar(absolutepath, files, config):
    # To place the legend later
    fig = plt.figure()
    ax = plt.subplot(111)

    bar_data = []
    bar_labels = []
    for f in files:
        line_n = 0
        data_array = []
        with open(absolutepath + '/' + f) as input_data:
            # Skips text before the beginning of the interesting block:
            for line in input_data:
                if line.strip() == 'Start of logging':  # Or whatever test is needed
                    break
            # Reads text until the end of the block:
            for line in input_data:  # This keeps reading the file
                if line.strip() == 'End of logging':
                    break
                line = line.rstrip("\n")
                line = float(line)
                data_array.append(line)
                line_n = line_n + 1
            bar_data.append(data_array)
            pos_a = f.rfind('.')
            bar_label = f[0:pos_a]
            bar_labels.append(bar_label)

    ind = (np.arange(line_n) + 1) * 5 / 60
    ax.stackplot(ind, bar_data, labels=bar_labels)
    plt.xlabel(config[0])
    plt.ylabel(config[1])
    plt.title(config[2])
    plt.legend(bbox_to_anchor=(1.04, 0.5), loc="center left", borderaxespad=0)
    plt.savefig(path+'/'+config[3], bbox_inches="tight")
    plt.show()


def plotlinechart(absolutepath, files, config):
    fig = plt.figure()
    ax = plt.subplot(111)

    t = np.arange(0, 5, float(5 / 60))
    for f in files:
        data_array = []
        with open(absolutepath + '/' + f) as input_data:
            # Skips text before the beginning of the interesting block:
            for line in input_data:
                if line.strip() == 'Start of logging':  # Or whatever test is needed
                    break
            # Reads text until the end of the block:
            for line in input_data:  # This keeps reading the file
                if line.strip() == 'End of logging':
                    break
                line = line.rstrip("\n")
                line = float(line)
                data_array.append(line)
            pos_a = f.rfind('.')
            f_label = f[0:pos_a]
            ax.plot(t, data_array, label=f_label)

    plt.xlabel(config[0])
    plt.ylabel(config[1])
    plt.title(config[2])
    plt.legend(bbox_to_anchor=(1.04, 0.5), loc="center left", borderaxespad=0)
    plt.savefig(path+'/'+config[3], bbox_inches="tight")
    plt.show()


files_array = seperatefiles(path);
if files_array[0]:
    if plottype == '1':
        linexylabeltitile = ['Time (min)', 'CPU Load (%)', 'CPU Load of Microservices', 'cpuloadLine.pdf']
        plotlinechart(path, files_array[0], linexylabeltitile)
    elif plottype == '2':
        barxylabeltitile = ['Time (min)', 'CPU Load (%)', 'CPU Load of Microservices', 'cpuloadBar.pdf']
        plotstackedbar(path, files_array[0], barxylabeltitile)
if files_array[1]:
    if plottype == '1':
        linexylabeltitile = ['Time (min)', 'Memory (%)', 'Memory of Microservices', 'memoryLine.pdf']
        plotstackedbar(path, files_array[1], linexylabeltitile)
    elif plottype == '2':
        barxylabeltitile = ['Time (min)', 'Memory (%)', 'Memory of Microservices', 'memoryBar.pdf']
        plotstackedbar(path, files_array[1], barxylabeltitile)

