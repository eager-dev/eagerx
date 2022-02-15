import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
import networkx as nx


def reset_graph(G):
    # Create a shallow copy graph that excludes feedthrough edges
    F = nx.MultiDiGraph(G)
    for u, v, key, data in G.edges(data=True, keys=True):
        if data["feedthrough"]:
            F.remove_edge(u, v, key=key)

    # Set actions node to stale
    if "observations" in F.nodes:
        F.nodes["observations"]["is_stale"] = True
    elif "env/observations" in F.nodes:
        F.nodes["env/observations"]["is_stale"] = True
    else:
        raise KeyError("Observation node not in graph.")
    return F


def episode_graph(G):
    # Create a shallow copy graph that excludes "skip" edges
    H = nx.MultiDiGraph(G)
    for u, v, key, data in G.edges(data=True, keys=True):
        if data["skip"]:
            H.remove_edge(u, v, key=key)

    # Color cyclic edges red
    cycles = []
    C = list(nx.simple_cycles(H))
    for c in C:
        H_sub = H.subgraph(c)
        for u, v, key, data in H_sub.edges(data=True, keys=True):
            G[u][v][key]["color"] = "red"
            data["color"] = "red"

        # Get all simple cyclic edge paths
        out_nodes = []
        for _u, v, _key, data in H_sub.out_edges(c[0], data=True, keys=True):
            out_nodes.append((v, data))

        for n, data_first in out_nodes:
            paths = nx.all_simple_edge_paths(H_sub, n, c[0])
            first_edge = (data_first["source"], data_first["target"])
            for path in paths:
                edges = [first_edge]
                for edge in path:
                    data = H_sub.get_edge_data(*edge)
                    edge = (data["source"], data["target"])
                    edges.append(edge)
                cycles.append(edges)
    return H, cycles


def plot_graph(G, ax=None, k=2, pos=None):
    if ax is None:
        env, ax = plt.subplots(nrows=1, ncols=1)

    # Determine position
    if pos is None:
        pos = nx.spring_layout(G, k=k)

    # Plot labels
    bbox = dict(facecolor="skyblue", edgecolor="black", boxstyle="round,pad=0.2", alpha=0.5)
    labels = nx.draw_networkx_labels(G, pos, bbox=bbox, ax=ax)
    for n, data in G.nodes(data=True):
        facecolor = data["facecolor"]
        labels[n].get_bbox_patch().set_facecolor(facecolor)

    # Plot nodes
    nodes_plt = nx.draw_networkx_nodes(G, pos, node_color="white", node_size=1500, alpha=0, ax=ax)

    # Plot edges
    colors = [data["color"] for u, v, data in G.edges(data=True)]
    styles = [data["style"] for u, v, data in G.edges(data=True)]
    alphas = [data["alpha"] for u, v, data in G.edges(data=True)]
    lines = nx.draw_networkx_edges(G, pos, arrows=True, edge_color=colors, width=2, ax=ax)
    for style, alpha, line in zip(styles, alphas, lines):
        line.set_linestyle(style)
        line.set_alpha(alpha)

    # Plot nodes
    nodes_plt = nx.draw_networkx_nodes(G, pos, node_color="white", node_size=350, alpha=1.0, ax=ax)

    # Create legend
    root_patch = mpatches.Patch(color="skyblue", label="source")
    sink_patch = mpatches.Patch(color="lightgreen", label="sink")
    stale_patch = mpatches.Patch(color="lightgrey", label="stale")
    node_patch = mpatches.Patch(color="khaki", label="node")
    stale_line = Line2D([0], [0], label="stale", color="lightgrey")
    cyclic_line = Line2D([0], [0], label="cyclic", color="red")
    dag_line = Line2D([0], [0], label="acyclic", color="black")
    nondag_line = Line2D([0], [0], label="skipped", color="green", linestyle="dotted")
    ax.legend(
        handles=[
            root_patch,
            sink_patch,
            node_patch,
            stale_patch,
            dag_line,
            cyclic_line,
            nondag_line,
            stale_line,
        ],
        ncol=4,
        prop={"size": 8},
        loc="upper center",
        bbox_to_anchor=(0.5, -0.05),
        fancybox=True,
        shadow=True,
    )
    return nodes_plt, lines, labels, pos


def color_nodes(G):
    # Color nodes based on in/out going edges
    for n, data in G.nodes(data=True):
        if data["is_stale"]:
            facecolor = "lightgrey"
        else:
            in_edges = G.in_degree(n)
            out_edges = G.out_degree(n)
            if in_edges == 0 and out_edges == 0:
                facecolor = "lightgrey"
            elif in_edges > 0 and out_edges == 0:
                facecolor = "lightgreen"
            elif in_edges == 0 and out_edges > 0:
                facecolor = "skyblue"
            else:  # in_edges > 0 and out_edges > 0:
                facecolor = "khaki"
        data["facecolor"] = facecolor


def color_edges(G):
    for _u, _v, _key, data in G.edges(data=True, keys=True):
        if data["is_stale"]:
            data["alpha"] = 0.3
        else:
            data["alpha"] = 1.0


def is_stale(G, exclude_skip=False):
    new_stale_nodes = [n for n, is_stale in nx.get_node_attributes(G, "is_stale").items() if is_stale]
    for n, data_n in G.nodes(data=True):
        # First, determine stale nodes, based on edges_in & always connected
        in_degree = G.in_degree(n)
        if in_degree == 0 and not data_n["always_active"] and not data_n["has_tick"]:
            data_n["is_stale"] = True
            new_stale_nodes.append(n)

        # Then, set all edges to active (=not stale)
        for _u, _v, _key, data_e in G.edges(n, data=True, keys=True):
            data_e["is_stale"] = False

    # Now, iteratively run over all new stale nodes and set out_edges and out_nodes to stale.
    # We add out_nodes (previously not stale) to new_stale nodes
    while len(new_stale_nodes) > 0:
        stale_nodes = new_stale_nodes
        new_stale_nodes = []

        # Then, loop over new stale nodes of prev. iteration and set all out-going edges to stale
        for n in stale_nodes:
            for u, v, _key, data_e in G.out_edges(n, data=True, keys=True):
                data_e["is_stale"] = True
                skip = data_e["skip"] if exclude_skip else False
                if u == "N8/out_1" and v == "N7/out_1":
                    print("wait")
                if not G.nodes[v]["is_stale"] and not skip:
                    if not G.nodes[v]["always_active"]:
                        new_stale_nodes.append(v)
                        G.nodes[v]["is_stale"] = True
    remain_active = [n for n, is_stale in nx.get_node_attributes(G, "remain_active").items() if is_stale]
    stale_nodes = [n for n, is_stale in nx.get_node_attributes(G, "is_stale").items() if is_stale]
    not_active = [n for n in stale_nodes if n in remain_active]
    return not_active
