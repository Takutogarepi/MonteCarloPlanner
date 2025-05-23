import pandas as pd
import matplotlib.pyplot as plt


def plot_graph(graph_title, x_label, y_label, tasks, mcts_time, hsp_time, filename):
    """
    We use this function to plot the graph. 

    """
    plt.figure()
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(graph_title)
    plt.plot(tasks,mcts_time,label = "MCTS")
    plt.plot(tasks, hsp_time, label="HSP")
    plt.legend()
    plt.savefig(filename)

def sort_domains(csv_data,domain_values,domain_names):
    """
    We use this function to sort the values.
    """
    sorted_values = csv_data[csv_data["domain"] == domain_names].sort_values(by="problem")
    for val in sorted_values.index:
        domain_values["mcts_time"].append(sorted_values.mcts_time[val])
        domain_values["mcts_length"].append(sorted_values.mcts_plan_length[val])
        domain_values["hsp_time"].append(sorted_values.hsp_time[val])
        domain_values["hsp_length"].append(sorted_values.hsp_plan_length[val])
        domain_values["problems"].append(str(sorted_values.problem[val]))

data = pd.read_csv("../../mcts_vs_hsp_comparison.csv", on_bad_lines="skip")
print(data.columns)

        
blocksworld = {
    "mcts_time" :[],
    "mcts_length" :[],
    "hsp_time" :[],
    "hsp_length" :[],
    "problems":[],

}


gripper = {
    "mcts_time" :[],
    "mcts_length" :[],
    "hsp_time" :[],
    "hsp_length" :[],
    "problems":[],

}

logistics = {
    "mcts_time" :[],
    "mcts_length" :[],
    "hsp_time" :[],
    "hsp_length" :[],
    "problems":[],

}

sort_domains(data,blocksworld,"blocks")
sort_domains(data,gripper,"gripper")
sort_domains(data,logistics,"logistics")

plot_graph("Runtime(ms) MCTS vs HSP blocks", "Task", "Runtime(ms)",blocksworld["problems"],blocksworld["mcts_time"],blocksworld["hsp_time"],"RuntimeMctsvsHspBlock.png")

plot_graph("Runtime(ms) MCTS vs HSP gripper", "Task", "Runtime(ms)",gripper["problems"],gripper["mcts_time"],gripper["hsp_time"],"RuntimeMctsvsHspGripper.png")

plot_graph("Runtime(ms) MCTS vs HSP logistics", "Task", "Runtime(ms)",logistics["problems"],logistics["mcts_time"],logistics["hsp_time"],"RuntimeMctsvsHspLogistics.png")

plot_graph("Plan length MCTS vs HSP blocks", "Task", "Plan length", blocksworld["problems"],blocksworld["mcts_length"],blocksworld["hsp_length"],"PlanLengthBlocks.png" )

plot_graph("Plan length MCTS vs HSP gripper", "Task", "Plan length", gripper["problems"],gripper["mcts_length"],gripper["hsp_length"],"PlanLengthGripper.png" )

plot_graph("Plan length MCTS vs HSP logistics", "Task", "Plan length", logistics["problems"],logistics["mcts_length"],logistics["hsp_length"],"PlanLengthLogistics.png" )



