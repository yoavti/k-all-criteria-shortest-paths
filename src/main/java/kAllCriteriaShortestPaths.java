import org.jgrapht.GraphPath;
import org.jgrapht.alg.flow.DinicMFImpl;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.*;

import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * Class containing the implementation of the solution to the k all-criteria-shortest paths problem
 * @param <T> The type of vertices
 */
public class kAllCriteriaShortestPaths<T> {

    public static void main(String[] args) {
        //maybe add testing here
    }

    /**
     * Algorithm solving the all-criteria-shortest disjoint k paths problem
     * @param G Directed Graph
     * @param weightFunctions q Weight Functions
     * @param s Source Node
     * @param t Target Node
     * @param k Integer
     * @return k Disjoint Paths
     * We assume that G contains s and t,
     * that all vertices are reachable from s,
     * that t is reachable from all vertices
     * and that there are no negative cycles in G w.r.t ant criterion
     */
    public Set<GraphPath<T, DefaultEdge>> kAllCriteriaShortestPathsAlg(DefaultDirectedGraph<T, DefaultEdge> G, Function<DefaultEdge, Double>[] weightFunctions, T s, T t, int k) {
        Function<DefaultEdge, Double> aggregatedWeight = e -> Arrays.stream(weightFunctions).map(f -> f.apply(e)).reduce(0.0, Double::sum);
        AsWeightedGraph<T, DefaultEdge> weightedGraph = new AsWeightedGraph<>(G, aggregatedWeight, false, false);
        return kSingleCriterionShortestPathsAlg(weightedGraph, s, t, k);
    }

    /**
     * Algorithm solving the single-criterion-shortest disjoint k paths problem
     * @param G Directed Weighted Graph
     * @param s Source Node
     * @param t Target Node
     * @param k Integer
     * @return k Disjoint Paths
     */
    private Set<GraphPath<T, DefaultEdge>> kSingleCriterionShortestPathsAlg(AsWeightedGraph<T, DefaultEdge> G, T s, T t, int k) {
        DijkstraShortestPath<T, DefaultEdge> dijkstra = new DijkstraShortestPath<>(G);
        double dst = dijkstra.getPath(s, t).getWeight(); //d(s,t)
        Set<T> vertices = G.vertexSet().stream().filter(v -> dst == dijkstra.getPath(s, v).getWeight() + dijkstra.getPath(v, t).getWeight()).collect(Collectors.toSet()); //All nodes that belong to at least one shortest path from s to t
        Set<DefaultEdge> edges = G.edgeSet().stream().filter(e -> dst == dijkstra.getPath(s, G.getEdgeSource(e)).getWeight() + G.getEdgeWeight(e) + dijkstra.getPath(G.getEdgeTarget(e), t).getWeight()).collect(Collectors.toSet()); //All edges that belong to at least one shortest path from s to t
        DefaultDirectedGraph<T, DefaultEdge> shortestPathGraph = new DefaultDirectedGraph<>(DefaultEdge.class);
        vertices.forEach(shortestPathGraph::addVertex);
        edges.forEach(e -> shortestPathGraph.addEdge(G.getEdgeSource(e), G.getEdgeTarget(e), e));
        return kDisjointPathsAlg(shortestPathGraph, s, t, k);
    }

    /**
     * Algorithm solving the k disjoint paths from s to t problem
     * @param G Directed Graph
     * @param s Source Node
     * @param t Target Node
     * @param k Integer
     * @return k Disjoint Paths
     */
    private Set<GraphPath<T, DefaultEdge>> kDisjointPathsAlg(DefaultDirectedGraph<T, DefaultEdge> G, T s, T t, int k) {
        AsWeightedGraph<T, DefaultEdge> weightedGraph = new AsWeightedGraph<>(G, e -> 1.0, false, false);
        DinicMFImpl<T, DefaultEdge> dinic = new DinicMFImpl<>(weightedGraph);
        dinic.dinic();
        if (dinic.getMaximumFlow(s, t).getValue() < k)
            return null;
        Map<DefaultEdge, Double> maxFlow = dinic.getFlowMap();
        Set<GraphPath<T, DefaultEdge>> P = new HashSet<>(); //returned set of k paths
        Stack<DefaultEdge> S = new Stack<>();
        Set<T> marked = new HashSet<>(); //Set of all marked edges
        for (int i = 0; i < k; i++) {
            //Phase 1
            T v = t;
            marked.add(v);
            while (v != s) {
                if (marked.contains(v)) {
                    //Phase 2
                    T z = v;
                    do {
                        DefaultEdge edge = S.pop();
                        v = weightedGraph.getEdgeSource(edge);
                        marked.remove(v);
                        maxFlow.put(edge, 0.0);
                    } while (v != z);
                    marked.add(v);
                }
                Set<DefaultEdge> Ef = weightedGraph.edgeSet().stream().filter(e -> maxFlow.get(e) == 1).collect(Collectors.toSet()); //Set of all edges with a flow of 1
                DefaultEdge edge = null;
                for (DefaultEdge e : Ef) {
                    if (weightedGraph.getEdgeTarget(e) == v) {
                        edge = e;
                        break;
                    }
                }
                S.push(edge);
                v = weightedGraph.getEdgeSource(edge);
                marked.add(v);
            }
            //Phase 3
            List<DefaultEdge> edgeList = new LinkedList<>();
            while (!S.empty()) {
                marked.remove(v);
                DefaultEdge edge = S.pop();
                v = weightedGraph.getEdgeSource(edge);
                marked.remove(v);
                maxFlow.put(edge, 0.0);
                edgeList.add(edge);
            }
            P.add(new GraphWalk<>(weightedGraph, s, t, edgeList, 0));
        }
        return P;
    }
}