import nested_dict as nd
from queue import PriorityQueue
from copy import deepcopy
from math import inf
import networkx as nx
import json

with open('json_graph1.json','r') as f:
    json_graph = json.load(f)

#Load graph and define LND riskfactor
G = nx.node_link_graph(json_graph)
LND_rf = 1.5*pow(10,-9)

#Compute cheapest paths between all possible senders and R for amount a. Paths are computed starting from the recipient.
# p1 contains previously computed cheapest paths for a different amount. The idea is that we do not need to compute paths
# again if we saw that the same path was computed for amounts a1<a<a2.
# S1 contains the list of senders for which we need to compute paths again.
def calc_paths_single_rcvr(G,R,a,p1,S1):
    paths = dict()
    paths1 = dict()
    wghts = dict()
    amts = dict()
    vstd = set()
    vstd1=set()
    for node in p1:
        #store paths and weights that need not be computed again.
        if node not in S1 and p1[node]!=[]:
            wght = 0
            amt = a
            flag = 0
            amts[p1[node][len(p1[node])-1]] = amt
            for i in range(len(p1[node])-2,0,-1):
                if G.edges[p1[node][i],p1[node][i+1]]['balance']+G.edges[p1[node][i+1],p1[node][i]]['balance']<=amt:
                    flag=1
                    break
                if p1[node][i] not in wghts:
                    wght = wght + LND_rf*G.edges[p1[node][i],p1[node][i+1]]['delay']*amt + G.edges[p1[node][i],p1[node][i+1]]['basefee'] + G.edges[p1[node][i],p1[node][i+1]]['feerate']*amt
                    amt = amt + G.edges[p1[node][i],p1[node][i+1]]['basefee'] + G.edges[p1[node][i],p1[node][i+1]]['feerate']*amt
                    paths[p1[node][i]] = p1[node][i:]
                    wghts[p1[node][i]] = wght
                    amts[p1[node][i]] = amt
                    vstd.add(p1[node][i])
            if len(p1[node])>=2 and flag==0:
                if G.edges[p1[node][0],p1[node][1]]['balance']>amts[p1[node][1]]:
                    paths1[node] = p1[node]
    #Initialize maximum weight for remaining nodes
    for node in G.nodes():
        if node not in wghts:
            wghts[node]=inf
    wghts[R] = 0
    amts[R] = a
    # Priority queue is used to efficiently compute dijkstra
    pq = PriorityQueue()
    paths[R] = [R]
    pq.put((wghts[R],R))
    # Find cheapest paths to all possible first intermediaries. We dont include senders here as the cost function for the sender is slightly different and does not include fees
    while (0 != pq.qsize()):
        curr_cost, curr = pq.get()
        if curr_cost > wghts[curr]:
            continue
        vstd.add(curr)
        for v in G.neighbors(curr):
            if (G.edges[v, curr]['balance'] + G.edges[curr, v]['balance'] >= amts[curr]) and v not in vstd: #Note that if we need to compute the path for v, it would already be in visited
                wght = wghts[curr] + LND_rf*G.edges[v,curr]['delay']*amts[curr] + G.edges[v,curr]['basefee']+G.edges[v,curr]['feerate']
                if wght < wghts[v]:
                    wghts[v] = wght
                    amts[v] = amts[curr] + G.edges[v, curr]['basefee'] + amts[curr] * \
                               G.edges[v, curr]['feerate']
                    pq.put((wghts[v], v))
                    paths[v] = [v] + paths[curr]
    # Find paths for all senders
    for S in G.nodes():
        if S not in paths1:
            wght = inf
            path = []
            for v in G.neighbors(S):
                if v in paths:
                    if S not in paths[v] and G.edges[S,v]['balance']>amts[v]:
                        wght1 = wghts[v] + LND_rf*G.edges[S,v]['delay']*amts[v]
                        if wght1<wght:
                            wght = wght1
                            path = [S]+paths[v]
            paths1[S] = path
    paths1[R] = [R]
    return paths1

#Here we find paths between all pairs of nodes and all amounts between a maximum and minimum value
def calc_all_paths(G,a1,a2):
    paths = nd.nested_dict()
    for R in G.nodes():
        print("Receiver:",R)
        p = nd.nested_dict()
        #Calculate paths for the max and min amounts
        p[a1] = calc_paths_single_rcvr(G,R,a1,{},[])
        p[a2] = calc_paths_single_rcvr(G,R,a2,{},[])
        intervals = nd.nested_dict()
        intervals[(a1,a2)] = []
        #check the senders for which the paths are different and store it corresponding to the interval (a1,a2)
        for S in p[a1]:
            if p[a1][S]==p[a2][S]:
                paths[R][S][(a1,a2)] = p[a1][S]
            else:
                intervals[(a1,a2)].append(S)
        if intervals[(a1,a2)]==[]:
            continue
        else:
            while(intervals!=None):
                intervals1 = nd.nested_dict()
                for interval in intervals:
                    intervals1[interval] = intervals[interval]
                # We repeat the iterations for smaller intervals till we exhaust the search and the difference between amounts in each interval is small(<1)
                for (b1,b2) in intervals1:
                    print(b1,b2,len(intervals1[(b1,b2)]))
                    b3 = (b1+b2)/2
                    del intervals[(b1,b2)]
                    if int(b2)-int(b1)<=1:
                        for S in intervals1[(b1,b2)]:
                            paths[R][S][(b1,b2)] = p[b1][S]
                        continue
                    intervals[(b1,b3)]=[]
                    intervals[(b3,b2)] = []
                    p[b3] = calc_paths_single_rcvr(G,R,b3,p[b1],intervals1[(b1,b2)])
                    for S in intervals1[(b1,b2)]:
                        if p[b1][S]==p[b3][S]:
                            paths[R][S][(b1,b3)] = p[b1][S]
                        else:
                            intervals[(b1,b3)].append(S)
                        if p[b3][S] == p[b2][S]:
                            paths[R][S][(b3, b2)] = p[b2][S]
                        else:
                            intervals[(b3, b2)].append(S)
                    if intervals[(b1,b3)]==[]:
                        del intervals[(b1,b3)]
                    if intervals[(b3,b2)]==[]:
                        del intervals[(b3,b2)]
    return paths

#sample run for a1=1 and a2=1000
p = calc_all_paths(G,1,1000)
