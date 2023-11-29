"""
 * DO NOT ALTER OR REMOVE THIS  NOTICES OR THIS FILE HEADER FROM THE CODE.
 * This code is free software you can redistribute it and/or modify it
   published by the Good will of the author.

 * This code is distributed in the hope that it will be useful,Therefore
   use this code for your need or purpose and you can inform Error or part
   to modify or add.
 * Modifiying this code in a part or as Whole is possible and incremental modification is
   suggested please inform the author any modification you have done.
 * Please contact the Author,if you need additional information or have any questions.
     *Author:Duressa Shukuri
     *Email:duressashukuri2022@gmail.com
"""
"""
   fromEdgeList,fromAdjecencyList,
   fromAdjecencyMatrix,fromAdjecencyMap 
   depthFirstSearch,breadthFirstSearch,
   Dijikastra,bellmann ,FolydWarshal
   prims,kruskal
   connected componenet,and topological Sorting
"""
import heapq
def fromEdgeList(edgelist):
    """
        converts adjecency edgelist into adjecency adjecency map
        params: edgelist
        return: Adjecency map
    """
    graph={}
    for edge in graph:
        [u,v]=edge
        if u not in graph:
            graph[u]=[]
        if v not in graph:
            graph[v]=[]
        graph[u].append(v)
        graph[v].append(u)
    return graph
def fromAdjecencyList(list):
    """
        converts adjecency list into adjecency adjecency map
        params: adjecency list
        return: Adjecency map
    """
    graph={}
    for node,neighbor in list:
        graph[node]=neighbor
    return graph
def fromAdjecencyMatrix(matrix):
    """
        converts adjecency matrix into adjecency adjecency map
        params: adjecency matrix
        return: Adjecency map
    """
    graph={}
    for i in range(len(matrix)):
        neighbor=[]
        for j in range(len(matrix)):
            if matrix[i][j]!=0:
                neighbor.append(matrix[i][j])
        graph[i]=neighbor
    return graph
def fromAdjecencyMap(map):
    """
        converts adjecency map into adjecency list
        params: adjecency map
        return: Adjecency list of nodes
    """
    listGraph=[]
    for vertex in map:
        listGraph.append((node,map[node]))
    return listGraph
def depthFirstSearch(graph,node,visted=set()):
    """
        perform depth first traversal by using recursion the prints the visted node on the console.
        params: graph,source,visted
        return: None
    """
    print(node,end="-->")
    visted.append(node)
    for neighbor in graph[node]:
        if neighbor not in visted:
            depthFirstSearch(graph,neighbor,visted)
def depthFirstSearch(graph,node,visted=set()):
    """
        perform depth first traversal by using stack the prints the visted node on the console.
        params: graph,source,visted
        return: None
    """
    stack=[node]
    while stack:
        current=stack.pop()
        if current not in visted:
            print(current,end="-->")
            visted.add(current)
        for neighbor in graph[current]:
            if neighbor not in visted:
                stack.append(neighbor)
def breadthFirstSearch(graph,node,visted=set()):
    """
        perform breadth first traversal by using queue the prints the visted node on the console.
        params: graph,source,visted
        return: None
    """
    queue=[node]
    while queue:
        current=queue.pop(0)
        if current not in visted:
            print(current,end="-->")
        for neighbor in graph[current]:
            if neighbor not in visted:
                queue.append(neighbor)
def dijikastraShortestPath(graph,source,visted=set()):
    """
        computes shortest distance betweeen source and all points of the graph and return them in the form of the map
        params: graph,source,visted
        return: map for shortest distance betweeen source node and  all nodes
    """
    distance={node:float("inf") for node in graph}
    distance[source]=0
    queue=[(0,source)]
    while queue:
        weight,node=heapq.heappop(queue)
        if node in visted:
            continue
        visted.add(node)
        for neighbor,weight in graph[node].items():
            if neighbor not in visted:
                current=distance[node]+weight
                if distance[neighbor]>current:
                    distance[neighbor]=current
                    heapq.heappush(queue,(current,neighbor))
    return distance
def bellmanFordShortestPath(graph,source):
    """
        computes shortest distance betweeen source and all points of the graph and return them in the form of the map
        params: graph,source
        return: map for shortest distance betweeen source node and  all nodes
    """
    distance={node:node for node in graph}
    distance[source]=0
    for edge in range(len(graph)-1):
        for u in graph:
            for v,weight in graph[u].items():
                if distance[v]>distance[u]+weight:
                    distance[v]=distance[u]+weight
    for u in graph:
        for v,weight in graph[u].items():
            if distance[v]>distance[u]+weight:
                return None
    return distance
def allPointShortestPath(graph):
    """
        computes shortest distance betweeen all points of the graph and return them in the form of the map
        params: graph
        return: map for shortest distance betweeen all nodes
    """
    distance={}
    def singlePointShortestPath(graph,source):
        distance={}
        for edge in range(len(graph) - 1):
            for u in graph:
                for v, weight in graph[u].items():
                    if distance[v] > distance[u] + weight:
                        distance[v] = distance[u] + weight
        for u in graph:
            for v, weight in graph[u].items():
                if distance[v] > distance[u] + weight:
                    return None
        return distance
    for node in graph:
        distance[node]=singlePointShortestPath(graph,node)
    return distance
def primsMinimumSpanninTree(graph,source,visted=set()):
    """
        Finds minimum spanning tree of the graph and return them in the form of the list
        params: graph,source
        return: return minimum spanning of  the graph
    """
    spanning=[]
    queue=[]
    for u,weight in graph[source].items():
        heapq.heappush(queue,(weight,source,u))
    while queue:
        weight,u,v=heapq.heappop(queue)
        if u in visted:
            continue
        spanning.append([u,v])
        for neighbor,weight in graph[v].items():
            if neighbor not in visted:
                heapq.heappush([weight,v,neighbor])
    return spanning
def kruskalMinimumSpanningTree(graph,source):
    """
        Finds minimum spanning tree of the graph and return them in the form of the list
        params: graph,source
        return: return minimum spanning of  the graph
    """
    spanning=[]
    queue=[]
    for u in graph:
        for v,weight in graph[u].items():
            heapq.heappush(queue,(weight,u,v))
    parent={node:node for node in graph}
    def getRoot(node):
        while node!=parent[node]:
            node=parent[node]
        return node
    def union(u,v):
        rootu=getRoot(u)
        rootv=getRoot(v)
        parent[rootv]=rootu
    while queue:
        weight,u,v=heapq.heappop(queue)
        if getRoot(u)!=getRoot(v):
            spanning.append([u,v])
            union(u,v)
    return spanning
def topologicalSorting(graph,visted=set()):
    """
        Finds topological ordering of  the graph and return them in the form of the list
        params: graph,source,visted
        return: topological ordering  of  the graph
    """
    sorting=[]
    def depthFirstSearch(graph,source,sorting,visted=set()):
        visted.add(source)
        for neighbor in graph[source]:
            if neighbor not in visted:
                depthFirstSearch(graph,neighbor,sorting,visted)
        sorting.insert(0,source)
    for node in graph:
        if node not in visted:
            result=[]
            depthFirstSearch(graph,node,result,visted)
            sorting.append(result)
    return sorting
def connectedComponent(graph,source,visted=set()):
    """
       Finds connected components of the graph and return them in the form of the list
       params: graph,source,visted
       return: return connected componenet of  the graph
    """
    component=[]
    def depthFirstSearch(graph,source,comp,visted=set()):
        comp.append(source)
        visted.add(source)
        for neighbor in graph:
            if neighbor not in visted:
                depthFirstSearch(graph,neighbor,comp,visted)
    for node in graph:
        if node not in visted:
            comp=[]
            depthFirstSearch(graph,node,comp,visted)
            component.append(comp)
    return component







