from math import gcd
from typing import List

class Task:
  def __init__(self, period, execution_time, deadline):
    self.p = period
    self.e = execution_time
    self.D = deadline

  def __repr__(self):
    return "({:.1f}, {:.1f})".format(self.p, self.e) if self.p == self.D \
      else "({:.1f}, {:.1f}, {:.1f})".format(self.p, self.e, self.D)

class Vertex:
  def __init__(self, name):
    self.name = name

  def __repr__(self):
    return self.name

class Edge:
  def __init__(self, start: Vertex, end: Vertex, capacity):
    self.start = start
    self.end = end
    self.capacity = capacity

class FlowGraph:
  def __init__(self):
    self.vertex_to_edges = {}
    self.edge_to_flow = {}
    self.source = self.addVertex("source")
    self.sink = self.addVertex("sink")

  def addVertex(self, name):
    v = Vertex(name)
    self.vertex_to_edges[v] = []
    return v

  def addEdge(self, a: Vertex, b: Vertex, capacity):
    edge = Edge(a, b, capacity)
    residual_edge = Edge(b, a, 0)

    edge.residual_edge = residual_edge
    residual_edge.residual_edge = edge

    self.vertex_to_edges[a].append(edge)
    self.vertex_to_edges[b].append(residual_edge)

    self.edge_to_flow[edge] = 0
    self.edge_to_flow[residual_edge] = 0

  def getEdges(self, vertex):
    return self.vertex_to_edges[vertex]

  def getFlow(self, edge):
    return self.edge_to_flow[edge]


def parseInput(lines):
  tasks = []
  for i in range(len(lines)):
    line = lines[i].split(',')
    deadline = line[2] if len(line) == 3 else line[0]
    task = Task(int(line[0]), int(line[1]), int(deadline))
    tasks.append(task)
  return tasks

def findHyperPeriod(tasks):
  def lcm(x, y):
    return (x * y) // gcd(x, y)

  periods = [t.p for t in tasks]
  res = periods.pop(0)
  while True:
    if len(periods) == 0:
      return res
    else:
      res = lcm(res, periods.pop(0))

def findFrameSizes(tasks, hyper_period):
  frames = []
  for f in range(2, hyper_period + 1):
    if hyper_period % f == 0:
      frames.append(f)

  # Frame size constraint 1) At least one i such that Pi / f == floor(Pi/ f) is satisfied
  divides_at_least_one_period = lambda f: any(task.p % f == 0 for task in tasks)
  frames = filter(divides_at_least_one_period, frames)

  # Frame size constraint 2) For all i, 2f - gcd(Pi, f) <= D is satisfied
  fits_all_periods = lambda f: all(2 * f - gcd(task.p, f) <= task.D for task in tasks)
  frames = filter(fits_all_periods, frames)

  return sorted(list(frames), reverse=True)

def constructFlowGraph(tasks: List[Task], hyper_period, frame_size):
  graph = FlowGraph()

  job_vertices = []
  for i in range(len(tasks)):
    task = tasks[i]
    for j in range(hyper_period // task.p):
      job_vertex = graph.addVertex("T{}_j{}".format(i + 1, j))
      job_vertex.task = i + 1
      job_vertex.release = task.p * j
      job_vertex.deadline = task.D * (j + 1)
      job_vertices.append(job_vertex)
      graph.addEdge(graph.source, job_vertex, tasks[i].e)

  frame_vertices = []
  for i in range(hyper_period // frame_size):
    frame_vertex = graph.addVertex("F{}".format(i))
    frame_vertex.start = i * frame_size
    frame_vertex.end = frame_vertex.start + frame_size
    frame_vertices.append(frame_vertex)
    graph.addEdge(frame_vertex, graph.sink, frame_size)

  for job_vertex in job_vertices:
    for frame_vertex in frame_vertices:
      if frame_vertex.start >= job_vertex.release and frame_vertex.end <= job_vertex.deadline:
        graph.addEdge(job_vertex, frame_vertex, frame_size)

  return graph

def dfs(graph: FlowGraph, start, end, path):
  if start == end: return path
  for edge in graph.getEdges(start):
    residual = edge.capacity - graph.getFlow(edge)
    if residual > 0 and not (edge, residual) in path:
      result = dfs(graph, edge.end, end, path + [(edge, residual)])
      if result is not None:
        return result

def computeMaxFlow(graph: FlowGraph):
  path = dfs(graph, graph.source, graph.sink, [])
  while path is not None:
    flow = min(residual for edge, residual in path)
    for edge, residual in path:
      graph.edge_to_flow[edge] += flow
      graph.edge_to_flow[edge.residual_edge] -= flow
    path = dfs(graph, graph.source, graph.sink, [])

  max_flow = 0
  for edge in graph.getEdges(graph.source):
    max_flow += graph.getFlow(edge)
  return max_flow

def computeTotalExecutionTime(tasks: List[Task], hyper_period):
  total = 0
  for i in range(len(tasks)):
    task = tasks[i]
    num_jobs = hyper_period / task.p
    total += num_jobs * task.e
  return total

def extractSchedule(graph: FlowGraph):
  job_slices = {}
  for vertex in graph.vertex_to_edges:
    is_job_vertex = hasattr(vertex, 'task')
    if is_job_vertex:
      edges = graph.getEdges(vertex)
      for edge in edges:
        end_vertex = edge.end
        is_frame_vertex = hasattr(end_vertex, 'start') and hasattr(end_vertex, 'end')
        if is_frame_vertex and graph.getFlow(edge) > 0:
          if vertex.name not in job_slices:
            job_slices[vertex.name] = []
          job_slices[vertex.name].append(float(graph.getFlow(edge)))

  task_jobs = {}
  for job_name in job_slices:
    task = job_name[1]
    if task not in task_jobs:
      task_jobs[task] = []
    task_jobs[task].append(job_slices[job_name])

  return task_jobs

def main():
  import sys

  lines = sys.stdin.readlines()
  tasks = parseInput(lines)
  print("Tasks:", tasks)

  H = findHyperPeriod(tasks)
  print("H =", float(H))

  frame_sizes = findFrameSizes(tasks, H)
  print("F =", frame_sizes)

  for f in frame_sizes:
    graph = constructFlowGraph(tasks, H, f)

    max_flow = computeMaxFlow(graph)
    total_exec_time = computeTotalExecutionTime(tasks, H)
    print("max_flow({}) =".format(f), float(max_flow))

    if max_flow == total_exec_time:
      task_jobs = extractSchedule(graph)
      for task in task_jobs:
        jobs = task_jobs[task]
        print("T_{}({}): F({})".format(task, len(jobs), str(jobs)))
    else:
      print("Unable to generate a schedule using frames of size", f)

if __name__ == '__main__':
  main()
