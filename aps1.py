import nested_dict as nd
from queue import PriorityQueue
from copy import deepcopy
from math import inf
import networkx as nx
import json

with open('json_graph1.json','r') as f:
    json_graph = json.load(f)

G = nx.node_link_graph(json_graph)
LND_rf = 1.5*pow(10,-9)

def calc_paths_single_rcvr(G,R,a,p1,S1):
    paths = dict()
    paths1 = dict()
    wghts = dict()
    amts = dict()
    vstd = set()
    vstd1=set()
    for node in p1:
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
    for node in G.nodes():
        if node not in wghts:
            wghts[node]=inf
    wghts[R] = 0
    amts[R] = a
    pq = PriorityQueue()
    paths[R] = [R]
    pq.put((wghts[R],R))
    while (0 != pq.qsize()):
        curr_cost, curr = pq.get()
        if curr_cost > wghts[curr]:
            continue
        vstd.add(curr)
        for v in G.neighbors(curr):
            if (G.edges[v, curr]['balance'] + G.edges[curr, v]['balance'] >= amts[curr]) and v not in vstd:
                wght = wghts[curr] + LND_rf*G.edges[v,curr]['delay']*amts[curr] + G.edges[v,curr]['basefee']+G.edges[v,curr]['feerate']
                if wght < wghts[v]:
                    wghts[v] = wght
                    amts[v] = amts[curr] + G.edges[v, curr]['basefee'] + amts[curr] * \
                               G.edges[v, curr]['feerate']
                    pq.put((wghts[v], v))
                    paths[v] = [v] + paths[curr]
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

def calc_all_paths(G,a1,a2):
    paths = nd.nested_dict()
    for R in G.nodes():
        print("Receiver:",R)
        p = nd.nested_dict()
        p[a1] = calc_paths_single_rcvr(G,R,a1,{},[])
        p[a2] = calc_paths_single_rcvr(G,R,a2,{},[])
        intervals = nd.nested_dict()
        intervals[(a1,a2)] = []
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
                for (b1,b2) in intervals1:
                    print(b1,b2,len(intervals1[(b1,b2)]))
                    b3 = (b1+b2)/2
                    del intervals[(b1,b2)]
                    if b2-b1<=1:
                        for S in intervals1[(b1,b2)]:
                            paths[R][S][(b1,b2)] = p[b1][S]
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

p = calc_all_paths(G,1,1000)