import org.jgrapht.Graph;
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
        //Examples on how to test the algorithm:
        //testAlgWithRandomGraph(3, 6, 1, 3, 3, 2, true, 2);
        //testAlgWithRandomGraphUntilSuccess(5, 15, 1, 3, 3, 2, true, 2);
    }

    /**
     * Test the implemented algorithm over a random graph and weight functions
     * @param n Number of nodes in the graph
     * @param m Number of edges in the graph
     * @param s Index of the starting node of the algorithm
     * @param t Index of the ending node of the algorithm
     * @param q Number of weight functions
     * @param maxWeight Max weight of the weight functions
     * @param round Whether or not to round the weights on each edge
     * @param k The index given in the algorithm
     */
    private static void testAlgWithRandomGraph(int n, int m, int s, int t, int q, int maxWeight, boolean round, int k) {
        if (s < 1 || s > n || t < 1 || t > n) {
            System.out.println("Invalid start node and/or end node given");
            return;
        }

        DefaultDirectedGraph<Integer, DefaultEdge> G = generateRandomConnectedDirectedGraph(n, m, s, t);
        if (G == null) {
            System.out.println("No such graph exists");
            return;
        }
        System.out.println("Input Graph: " + G);

        System.out.println("Weight Functions:");
        List<Function<DefaultEdge, Double>> weightFunctions = generateRandomWeightFunctions(G, q, maxWeight, round);
        printWeightFunctions(weightFunctions, G);

        Set<GraphPath<Integer, DefaultEdge>> res = new kAllCriteriaShortestPaths<Integer, DefaultEdge>().kAllCriteriaShortestPathsAlg(G, weightFunctions, s, t, k);
        if (res == null) {
            System.out.println("No solution exists");
            return;
        }
        System.out.println("Result: " + res);
    }

    /**
     * Randomly generate an input graph and weight functions until they yield a solution to the problem.
     * Some inputs to this function will never yield a solution to the problem,
     * so one should be cautious when calling this function, as it may cause an infinite loop
     * @param n Number of nodes in the graph
     * @param m Number of edges in the graph
     * @param s Index of the starting node of the algorithm
     * @param t Index of the ending node of the algorithm
     * @param q Number of weight functions
     * @param maxWeight Max weight of the weight functions
     * @param round Whether or not to round the weights on each edge
     * @param k The index given in the algorithm
     */
    private static void testAlgWithRandomGraphUntilSuccess(int n, int m, int s, int t, int q, int maxWeight, boolean round, int k) {
        if (s < 1 || s > n || t < 1 || t > n) {
            System.out.println("Invalid start node and/or end node given");
            return;
        }

        int attempt = 0;
        kAllCriteriaShortestPaths<Integer, DefaultEdge> alg = new kAllCriteriaShortestPaths<>();
        DefaultDirectedGraph<Integer, DefaultEdge> G = generateRandomConnectedDirectedGraph(n, m, s, t);
        if (G == null) { //Making sure a valid graph can be created for the given n and m
            System.out.println("No such graph exists");
            return;
        }
        List<Function<DefaultEdge, Double>> weightFunctions;
        Set<GraphPath<Integer, DefaultEdge>> res = null;

        while (res == null) {
            attempt++;
            System.out.println("Attempt " + attempt);

            G = generateRandomConnectedDirectedGraph(n, m, s, t);
            System.out.println("Input Graph: " + G);

            System.out.println("Weight Functions:");
            weightFunctions = generateRandomWeightFunctions(G, q, maxWeight, round);
            printWeightFunctions(weightFunctions, G);

            res = alg.kAllCriteriaShortestPathsAlg(G, weightFunctions, s, t, k);
        }

        System.out.println("Result: " + res);
    }

    /**
     * Prints the received weight functions in a pretty manner
     * @param weightFunctions The weight functions to print
     * @param G The graph the weight functions exist in
     */
    private static void printWeightFunctions(List<Function<DefaultEdge, Double>> weightFunctions, Graph<Integer, DefaultEdge> G) {
        int ind = 1;
        for (Function<DefaultEdge, Double> weightFunction : weightFunctions) {
            for (DefaultEdge e : G.edgeSet()) {
                System.out.println("w" + ind + "(" + e + ") = " + weightFunction.apply(e));
            }
            System.out.println();
            ind++;
        }
    }

    /**
     * Helper function for main. Used for generating a random directed graph
     * @param n Number of vertices
     * @param m Number of edges
     * @return A random directed graph with Integers as vertices and DefaultEdges as edges with n vertices and m edges
     */
    private static DefaultDirectedGraph<Integer, DefaultEdge> generateRandomDirectedGraph(int n, int m) {
        if (n < 1 || m < n || m > n * (n - 1)) {
            return null;
        }

        DefaultDirectedGraph<Integer, DefaultEdge> ret = new DefaultDirectedGraph<>(DefaultEdge.class);

        //Adding vertices
        for (int i = 1; i <= n; i++) {
            ret.addVertex(i);
        }

        //Adding edges
        Random random = new Random();
        int to, from;
        boolean added;
        for (int i = 1; i <= m; i++) {
            added = false;
            while (!added) {
                to = 0;
                from = 0;
                while (to == from) {
                    to = random.nextInt(n) + 1;
                    from = random.nextInt(n) + 1;
                }
                added = ret.addEdge(from, to) != null;
            }
        }

        return ret;
    }

    /**
     * Helper function for main. Used for generating a random connected directed graph
     * @param n Number of vertices
     * @param m Number of edges
     * @return A random connected directed graph with Integers as vertices and DefaultEdges as edges with n vertices and m edges
     */
    private static DefaultDirectedGraph<Integer, DefaultEdge> generateRandomConnectedDirectedGraph(int n, int m, int s, int t) {
        boolean connected = false;
        DefaultDirectedGraph<Integer, DefaultEdge> ret = null;
        while (!connected) {
            ret = generateRandomDirectedGraph(n, m);
            if (ret == null) {
                return null;
            }
            connected = isConnected(ret, s, t);
        }
        return ret;
    }

    /**
     * Used to guarantee the preconditions for the input graph of the algorithm
     * @param G A graph
     * @param s Start node
     * @param t End node
     * @return Whether all nodes are reachable from s and all nodes reach t
     */
    private static boolean isConnected(Graph<Integer, DefaultEdge> G, int s, int t) {
        DijkstraShortestPath<Integer, DefaultEdge> dijkstra = new DijkstraShortestPath<>(new AsWeightedGraph<>(G, e -> 1.0, false, false));
        for (Integer v : G.vertexSet()) {
            if (dijkstra.getPath(s, v) == null || dijkstra.getPath(v, t) == null) {
                return false;
            }
        }
        return true;
    }

    /**
     * Helper function for main. Used for generating q random weight functions
     * @param G Graph the weight functions exist in
     * @param q Number of weight functions to generate
     * @param maxWeight Maximum weight for a single edge in any weight functions
     * @param round Whether to round weight or not
     * @return A list of q weight functions
     * All weight functions are non-negative
     */
    private static List<Function<DefaultEdge, Double>> generateRandomWeightFunctions(Graph<Integer, DefaultEdge> G, int q, int maxWeight, boolean round) {
        Random random = new Random();
        List<Map<DefaultEdge, Double>> maps = new LinkedList<>();
        for (int i = 0; i < q; i++) {
            maps.add(new HashMap<>());
        }
        for (Map<DefaultEdge, Double> map : maps) {
            for (DefaultEdge e : G.edgeSet()) {
                map.put(e, round ? Math.floor(random.nextDouble() * maxWeight) : random.nextDouble() * maxWeight);
            }
        }
        List<Function<DefaultEdge, Double>> weightFunctions = new LinkedList<>();
        for (Map<DefaultEdge, Double> map : maps) {
            weightFunctions.add(map::get);
        }
        return weightFunctions;
    }

    /**
     * Algorithm solving the all-criteria-shortest disjoint k paths problem
     * @param G Directed Graph
     * @param weightFunctions q Weight Functions
     * @param s Source Node
     * @param t Target Node
     * @param k Integer
     * @return k Disjoint Paths or null if none exist
     * We assume that G contains s and t,
     * that all vertices are reachable from s,
     * that t is reachable from all vertices,
     * and that there are no negative cycles in G w.r.t ant criterion
     */
    public Set<GraphPath<V, E>> kAllCriteriaShortestPathsAlg(DefaultDirectedGraph<V, E> G, List<Function<E, Double>> weightFunctions, V s, V t, int k) {
        Function<E, Double> aggregatedWeight = e -> weightFunctions.stream().map(f -> f.apply(e)).reduce(0.0, Double::sum); //The aggregated weight function, as defined in the paper
        AsWeightedGraph<V, E> weightedGraph = new AsWeightedGraph<>(G, aggregatedWeight, false, false); //The graph after applying the aggregated function to its edges
        DijkstraShortestPath<V, E> dijkstra = new DijkstraShortestPath<>(weightedGraph);
        double dst = dijkstra.getPath(s, t).getWeight(); //d(s,t)
        double sumDists = 0.0;
        for (Function<E, Double> weightFunction : weightFunctions) {
            sumDists += new DijkstraShortestPath<>(new AsWeightedGraph<>(G, weightFunction, false, false)).getPath(s, t).getWeight();
        }
        return sumDists == dst ? kSingleCriterionShortestPathsAlg(weightedGraph, s, t, k) : null;
    }

    /**
     * Algorithm solving the single-criterion-shortest disjoint k paths problem
     * @param G Directed Weighted Graph
     * @param s Source Node
     * @param t Target Node
     * @param k Integer
     * @return k Disjoint Paths or null if none exist
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
     * @return k Disjoint Paths or null if none exist
     */
    private Set<GraphPath<V, E>> kDisjointPathsAlg(AsSubgraph<V, E> G, V s, V t, int k) {
        AsWeightedGraph<V, E> weightedGraph = new AsWeightedGraph<>(G, e -> 1.0, false, false); //Flow network with unit capacities on all of its edges
        DinicMFImpl<V, E> dinic = new DinicMFImpl<>(weightedGraph);
        if (dinic.getMaximumFlow(s, t).getValue() < k) {
            return null;
        }
        Map<E, Double> maxFlow = dinic.getFlowMap();
        Set<E> Ef = G.edgeSet().stream().filter(e -> maxFlow.get(e) == 1).collect(Collectors.toSet()); //Set of all edges with a flow of 1
        Set<GraphPath<V, E>> P = new HashSet<>(); //returned set of k paths
        Stack<E> S = new Stack<>(); //Stack used in the path finding routine
        Set<V> marked = new HashSet<>(); //Set of all marked vertices
        for (int i = 0; i < k; i++) {
            //Phase 1
            while (!S.empty()) {
                S.pop(); //Empty the stack
            }
            V v = t;
            marked.add(v);
            while (!v.equals(s)) {
                E edge = null;
                Set<E> intersection = new HashSet<>(G.incomingEdgesOf(v));
                intersection.retainAll(Ef);
                for (E e: intersection) {
                    edge = e;
                    break;
                }
                S.push(edge);
                v = G.getEdgeSource(edge);
                if (marked.contains(v)) {
                    //Phase 2
                    V z = v;
                    do {
                        E e = S.pop();
                        v = G.getEdgeSource(e);
                        marked.remove(v);
                        Ef.remove(e);
                    } while (!v.equals(z));
                    marked.add(v);
                }
                marked.add(v);
            }
            //Phase 3
            List<E> edgeList = new LinkedList<>(); //List of edges of the path to be added to P
            while (!v.equals(t)) {
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