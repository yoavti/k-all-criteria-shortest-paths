import org.jgrapht.GraphPath;
import org.jgrapht.alg.flow.DinicMFImpl;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.*;

import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * Class containing the implementation of the solution to the k all-criteria-shortest paths problem
 * @param <V> Type of vertices
 * @param <E> Type of edges
 */
public class kAllCriteriaShortestPaths<V, E> {

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
     * that t is reachable from all vertices,
     * and that there are no negative cycles in G w.r.t ant criterion
     */
    public Set<GraphPath<V, E>> kAllCriteriaShortestPathsAlg(DefaultDirectedGraph<V, E> G, Function<E, Double>[] weightFunctions, V s, V t, int k) {
        Function<E, Double> aggregatedWeight = e -> Arrays.stream(weightFunctions).map(f -> f.apply(e)).reduce(0.0, Double::sum); //The aggregated weight function, as defined in the paper
        AsWeightedGraph<V, E> weightedGraph = new AsWeightedGraph<>(G, aggregatedWeight, false, false); //The graph after applying the aggregated function to its edges
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
    private Set<GraphPath<V, E>> kSingleCriterionShortestPathsAlg(AsWeightedGraph<V, E> G, V s, V t, int k) {
        DijkstraShortestPath<V, E> dijkstra = new DijkstraShortestPath<>(G);
        double dst = dijkstra.getPath(s, t).getWeight(); //d(s,t)
        Set<V> vertices = G.vertexSet().stream().filter(v -> dst == dijkstra.getPath(s, v).getWeight() + dijkstra.getPath(v, t).getWeight()).collect(Collectors.toSet()); //All nodes that belong to at least one shortest path from s to t
        Set<E> edges = G.edgeSet().stream().filter(e -> dst == dijkstra.getPath(s, G.getEdgeSource(e)).getWeight() + G.getEdgeWeight(e) + dijkstra.getPath(G.getEdgeTarget(e), t).getWeight()).collect(Collectors.toSet()); //All edges that belong to at least one shortest path from s to t
        AsSubgraph<V, E> shortestPathGraph = new AsSubgraph<>(G, vertices, edges); //The subgraph containing vertices and edges belonging to shortest paths in the original graph
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
    private Set<GraphPath<V, E>> kDisjointPathsAlg(AsSubgraph<V, E> G, V s, V t, int k) {
        AsWeightedGraph<V, E> weightedGraph = new AsWeightedGraph<>(G, e -> 1.0, false, false); //Flow network with unit capacities on all of its edges
        DinicMFImpl<V, E> dinic = new DinicMFImpl<>(weightedGraph);
        dinic.dinic();
        if (dinic.getMaximumFlow(s, t).getValue() < k)
            return null;
        Map<E, Double> maxFlow = dinic.getFlowMap();
        Set<E> Ef = G.edgeSet().stream().filter(e -> maxFlow.get(e) == 1).collect(Collectors.toSet()); //Set of all edges with a flow of 1
        Set<GraphPath<V, E>> P = new HashSet<>(); //returned set of k paths
        Stack<E> S = new Stack<>(); //Stack used in the path finding routine
        Set<V> marked = new HashSet<>(); //Set of all marked edges
        for (int i = 0; i < k; i++) {
            //Phase 1
            while (!S.empty())
                S.pop(); //Empty the stack
            V v = t;
            marked.add(v);
            while (v != s) {
                if (marked.contains(v)) {
                    //Phase 2
                    V z = v;
                    do {
                        E edge = S.pop();
                        v = G.getEdgeSource(edge);
                        marked.remove(v);
                        Ef.remove(edge);
                    } while (v != z);
                    marked.add(v);
                }
                E edge = null;
                for (E e : Ef) {
                    if (G.getEdgeTarget(e) == v) {
                        edge = e;
                        break;
                    }
                }
                S.push(edge);
                v = G.getEdgeSource(edge);
                marked.add(v);
            }
            //Phase 3
            List<E> edgeList = new LinkedList<>(); //List of edges of the path to be added to P
            while (v != t) {
                marked.remove(v);
                E edge = S.pop();
                v = G.getEdgeTarget(edge);
                marked.remove(v);
                Ef.remove(edge);
                edgeList.add(edge);
            }
            P.add(new GraphWalk<>(G, s, t, edgeList, 0));
        }
        return P;
    }
}