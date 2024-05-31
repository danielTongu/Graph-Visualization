import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.io.*;
import java.util.*;
import java.util.List;
import java.util.Objects;

/**
 * The Edge class represents an edge in a graph with a source, destination, and weight.
 * @param <U> the type of the source vertex
 * @param <V> the type of the destination vertex
 *
 * @author Daniel Tongu
 */
final class Edge<U, V> implements Serializable {
	final U SOURCE;
	final V DESTINATION;
	double weight;

	/**
	 * Constructs an Edge with the specified source, destination, and weight.
	 * @param source      the source vertex
	 * @param destination the destination vertex
	 * @param weight      the weight of the edge
	 */
	public Edge(U source, V destination, double weight) {
		this.SOURCE = source;
		this.DESTINATION = destination;
		this.weight = weight;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		Edge<?, ?> edge = (Edge<?, ?>) o;
		return Double.compare(edge.weight, weight) == 0 &&
				Objects.equals(SOURCE, edge.SOURCE) &&
				Objects.equals(DESTINATION, edge.DESTINATION);
	}

	@Override
	public int hashCode() {
		return Objects.hash(SOURCE, DESTINATION, weight);
	}

	@Override
	public String toString() {
		return "Edge{" +
				"source=" + SOURCE +
				", destination=" + DESTINATION +
				", weight=" + weight +
				'}';
	}
}

/**
 * The Graph class represents an abstract graph structure.
 * @param <T> the type of the vertices in the graph
 *
 * @author Daniel Tongu
 */
public abstract class Graph<T> implements Serializable {
	protected final Map<T, Set<Edge<T, T>>> vertices = new HashMap<>();

	/**
	 * Copies the vertices and edges from the specified graph into this graph.
	 * @param other the graph to copy from
	 */
	public void copyFrom(Graph<T> other) {
		this.vertices.clear();
		other.vertices.forEach((vertex, edges) -> {
			Set<Edge<T, T>> newEdges = new HashSet<>();
			for (Edge<T, T> edge : edges) {
				newEdges.add(new Edge<>(vertex, edge.DESTINATION, edge.weight));
			}
			this.vertices.put(vertex, newEdges);
		});
	}

	/**
	 * Returns the minimum spanning tree of the graph starting from the specified vertex.
	 * @param start the starting vertex
	 * @return the minimum spanning tree
	 */
	public abstract Graph<T> minimumSpanningTree(T start);

	/**
	 * Returns the degree of the specified vertex.
	 * @param vertex the vertex
	 * @return the degree of the vertex
	 */
	public int getDegree(T vertex) {
		return vertices.getOrDefault(vertex, Collections.emptySet()).size();
	}

	/**
	 * Returns whether the graph is directed.
	 * @return true if the graph is directed, false otherwise
	 */
	public abstract boolean isDirected();

	/**
	 * Returns whether the graph is empty.
	 * @return true if the graph is empty, false otherwise
	 */
	public boolean isEmpty() {
		return vertices.isEmpty();
	}

	/**
	 * Returns whether the graph contains the specified vertex.
	 * @param vertex the vertex
	 * @return true if the graph contains the vertex, false otherwise
	 */
	public boolean containsVertex(T vertex) {
		return vertices.containsKey(vertex);
	}

	/**
	 * Adds the specified vertex to the graph.
	 * @param vertex the vertex to add
	 * @return true if the vertex was added, false if it already existed
	 */
	public boolean addVertex(T vertex) {
		if (vertices.containsKey(vertex)) {
			return false;
		}
		vertices.put(vertex, new HashSet<>());
		return true;
	}

	/**
	 * Removes the specified vertex from the graph.
	 * @param vertex the vertex to remove
	 * @return true if the vertex was removed, false if it did not exist
	 */
	public boolean removeVertex(T vertex) {
		if (!vertices.containsKey(vertex)) {
			return false;
		}
		vertices.values().forEach(edges -> edges.removeIf(edge -> edge.DESTINATION.equals(vertex)));
		vertices.remove(vertex);
		return true;
	}

	/**
	 * Returns a list of all vertices in the graph.
	 * @return a list of all vertices
	 */
	public List<T> getVertices() {
		return new ArrayList<>(vertices.keySet());
	}

	/**
	 * Returns whether the specified vertices are neighbors.
	 * @param v1 the first vertex
	 * @param v2 the second vertex
	 * @return true if the vertices are neighbors, false otherwise
	 */
	public boolean areNeighbors(T v1, T v2) {
		return getVertexEdges(v1).stream().anyMatch(edge -> edge.DESTINATION.equals(v2)) ||
				getVertexEdges(v2).stream().anyMatch(edge -> edge.DESTINATION.equals(v1));
	}

	/**
	 * Adds an edge between the specified vertices with the given weight.
	 * @param v1     the first vertex
	 * @param v2     the second vertex
	 * @param weight the weight of the edge
	 * @return true if the edge was added, false if it already existed
	 */
	public abstract boolean addEdge(T v1, T v2, double weight);

	/**
	 * Removes the edge between the specified vertices.
	 * @param v1 the first vertex
	 * @param v2 the second vertex
	 * @return true if the edge was removed, false if it did not exist
	 */
	public abstract boolean removeEdge(T v1, T v2);

	/**
	 * Returns whether there is an edge between the specified vertices.
	 * @param v1 the first vertex
	 * @param v2 the second vertex
	 * @return true if there is an edge between the vertices, false otherwise
	 */
	public boolean containsEdge(T v1, T v2) {
		if (vertices.containsKey(v1)) {
			return vertices.get(v1).stream().anyMatch(edge -> edge.DESTINATION.equals(v2));
		}
		return false;
	}

	/**
	 * Determines whether an edge should be added between the specified vertices.
	 * @param v1 the first vertex
	 * @param v2 the second vertex
	 * @return true if the edge should be added, false otherwise
	 */
	protected abstract boolean shouldAddEdge(T v1, T v2);

	/**
	 * Returns a list of all edges in the graph.
	 * @return a list of all edges
	 */
	public List<Edge<T, T>> getEdges() {
		List<Edge<T, T>> edges = new ArrayList<>();
		vertices.forEach((v1, connectedEdges) -> connectedEdges.forEach(edge -> {
			if (shouldAddEdge(v1, edge.DESTINATION)) {
				edges.add(new Edge<>(v1, edge.DESTINATION, edge.weight));
			}
		}));
		return edges;
	}

	/**
	 * Returns the weight of the edge between the specified vertices.
	 * @param t1 the first vertex
	 * @param t2 the second vertex
	 * @return the weight of the edge, or NaN if no edge exists
	 */
	public double getEdgeWeight(T t1, T t2) {
		double weight = checkEdgeWeight(t1, t2);
		if (!Double.isNaN(weight)) {
			return weight;
		}
		return checkEdgeWeight(t2, t1);
	}

	/**
	 * Checks the weight of the edge from the source vertex to the destination vertex.
	 * @param source the source vertex
	 * @param dest   the destination vertex
	 * @return the weight of the edge, or NaN if no edge exists
	 */
	private double checkEdgeWeight(T source, T dest) {
		if (vertices.containsKey(source)) {
			for (Edge<T, T> edge : vertices.get(source)) {
				if (edge.DESTINATION.equals(dest)) {
					return edge.weight;
				}
			}
		}
		return Double.NaN;
	}

	/**
	 * Returns the set of edges connected to the specified vertex.
	 * @param vertex the vertex
	 * @return the set of edges connected to the vertex
	 */
	public Set<Edge<T, T>> getVertexEdges(T vertex) {
		return vertices.getOrDefault(vertex, Collections.emptySet());
	}

	/**
	 * Performs a depth-first search (DFS) starting from the specified vertex.
	 * @param startVertex the starting vertex
	 * @return a list of edges in the order they were visited
	 */
	public List<Edge<T, T>> dfs(T startVertex) {
		List<Edge<T, T>> edgeList = new ArrayList<>();
		if (!vertices.containsKey(startVertex)) {
			return edgeList;
		}

		Stack<Edge<T, T>> stack = new Stack<>();
		Set<T> seen = new HashSet<>();

		seen.add(startVertex);
		List<Edge<T, T>> edges = new ArrayList<>(vertices.get(startVertex));
		edges.sort(Comparator.comparingDouble(e -> e.hashCode()).reversed());
		for (Edge<T, T> edge : edges) {
			stack.push(edge);
		}

		while (!stack.isEmpty()) {
			Edge<T, T> edge = stack.pop();
			T vertex = edge.DESTINATION;
			if (!seen.contains(vertex)) {
				seen.add(vertex);
				edgeList.add(edge);
				edges = new ArrayList<>(vertices.get(vertex));
				edges.sort(Comparator.comparingDouble(e -> e.hashCode()).reversed());
				for (Edge<T, T> e : edges) {
					stack.push(e);
				}
			}
		}
		return edgeList;
	}

	/**
	 * Performs a breadth-first search (BFS) starting from the specified vertex.
	 * @param startVertex the starting vertex
	 * @return a list of edges in the order they were visited
	 */
	public List<Edge<T, T>> bfs(T startVertex) {
		List<Edge<T, T>> edgeList = new ArrayList<>();
		if (!vertices.containsKey(startVertex)) {
			return edgeList;
		}
		Queue<T> queue = new LinkedList<>();
		queue.add(startVertex);
		Set<T> seen = new HashSet<>();
		seen.add(startVertex);

		while (!queue.isEmpty()) {
			T current = queue.poll();
			List<Edge<T, T>> edges = new ArrayList<>(vertices.get(current));
			edges.sort(Comparator.comparingInt(edge -> edge.DESTINATION.hashCode()));
			for (Edge<T, T> edge : edges) {
				T neighbor = edge.DESTINATION;
				if (!seen.contains(neighbor)) {
					seen.add(neighbor);
					queue.add(neighbor);
					edgeList.add(edge);
				}
			}
		}
		return edgeList;
	}

	/**
	 * Returns the shortest path from the start vertex to the end vertex.
	 * @param startVertex the starting vertex
	 * @param endVertex   the ending vertex
	 * @return a list of edges representing the shortest path
	 */
	public List<Edge<T, T>> shortestPath(T startVertex, T endVertex) {
		List<Edge<T, T>> edgeList = new ArrayList<>();
		if (!vertices.containsKey(startVertex) || !vertices.containsKey(endVertex)) {
			return edgeList;
		}

		Map<T, T> predecessors = new HashMap<>();
		Map<T, Double> distances = new HashMap<>();
		PriorityQueue<T> queue = new PriorityQueue<>(Comparator.comparing(distances::get));
		vertices.keySet().forEach(v -> distances.put(v, Double.MAX_VALUE));
		distances.put(startVertex, 0.0);
		queue.add(startVertex);

		while (!queue.isEmpty()) {
			T current = queue.poll();
			if (current.equals(endVertex)) {
				break;
			}

			for (Edge<T, T> edge : vertices.get(current)) {
				T neighbor = edge.DESTINATION;
				double newDistance = distances.get(current) + edge.weight;
				if (newDistance < distances.get(neighbor)) {
					distances.put(neighbor, newDistance);
					predecessors.put(neighbor, current);
					queue.add(neighbor);
				}
			}
		}

		return reconstructEdgePath(predecessors, startVertex, endVertex);
	}

	/**
	 * Reconstructs the path of edges from the predecessors map.
	 * @param predecessors the map of predecessors
	 * @param startVertex  the starting vertex
	 * @param endVertex    the ending vertex
	 * @return a list of edges representing the path
	 */
	private List<Edge<T, T>> reconstructEdgePath(Map<T, T> predecessors, T startVertex, T endVertex) {
		LinkedList<Edge<T, T>> path = new LinkedList<>();
		T step = endVertex;
		if (predecessors.get(step) == null) {
			return path; // no path found
		}
		while (!step.equals(startVertex)) {
			T predecessor = predecessors.get(step);
			path.addFirst(findEdge(predecessor, step));
			step = predecessor;
		}
		return path;
	}

	/**
	 * Finds the edge between the specified vertices.
	 * @param v1 the first vertex
	 * @param v2 the second vertex
	 * @return the edge, or null if no edge exists
	 */
	private Edge<T, T> findEdge(T v1, T v2) {
		return getVertexEdges(v1).stream().filter(edge -> edge.DESTINATION.equals(v2)).findFirst().orElse(null);
	}
}

/**
 * The UndirectedGraph class represents an undirected graph.
 * @param <T> the type of the vertices in the graph
 *
 * @author Daniel Tongu
 */
class UndirectedGraph<T> extends Graph<T> {
	@Override
	public boolean isDirected() {
		return false;
	}

	@Override
	public Graph<T> minimumSpanningTree(T start) {
		if (!super.vertices.containsKey(start)) {
			throw new IllegalArgumentException("Start vertex does not exist in the graph.");
		}

		UndirectedGraph<T> mst = new UndirectedGraph<>();
		mst.addVertex(start);

		PriorityQueue<Edge<T, T>> edgeQueue = new PriorityQueue<>(Comparator.comparingDouble(Edge::hashCode));
		Set<T> added = new HashSet<>();
		added.add(start);

		edgeQueue.addAll(super.vertices.get(start));

		while (!edgeQueue.isEmpty()) {
			Edge<T, T> edge = edgeQueue.poll();
			if (added.contains(edge.DESTINATION)) continue;

			mst.addVertex(edge.DESTINATION);
			mst.addEdge(edge.SOURCE, edge.DESTINATION, edge.weight);
			added.add(edge.DESTINATION);

			for (Edge<T, T> newEdge : super.vertices.get(edge.DESTINATION)) {
				if (!added.contains(newEdge.DESTINATION)) {
					edgeQueue.add(newEdge);
				}
			}
		}

		return mst;
	}

	@Override
	public boolean addEdge(T v1, T v2, double weight) {
		if (v1.equals(v2)) {
			throw new IllegalArgumentException("No self-loops allowed.");
		}
		super.vertices.putIfAbsent(v1, new HashSet<>());
		super.vertices.putIfAbsent(v2, new HashSet<>());
		boolean addedV1 = super.vertices.get(v1).add(new Edge<>(v1, v2, weight));
		boolean addedV2 = super.vertices.get(v2).add(new Edge<>(v2, v1, weight));
		return addedV1 && addedV2;
	}

	@Override
	public boolean removeEdge(T v1, T v2) {
		boolean removedV1 = super.vertices.containsKey(v1) && super.vertices.get(v1).removeIf(edge -> edge.DESTINATION.equals(v2));
		boolean removedV2 = super.vertices.containsKey(v2) && super.vertices.get(v2).removeIf(edge -> edge.DESTINATION.equals(v1));
		return removedV1 && removedV2;
	}

	@Override
	protected boolean shouldAddEdge(T v1, T v2) {
		return v1.hashCode() <= v2.hashCode();
	}
}

/**
 * The DirectedGraph class represents a directed graph.
 * @param <T> the type of the vertices in the graph
 */
class DirectedGraph<T> extends Graph<T> {
	@Override
	public boolean isDirected() {
		return true;
	}

	@Override
	public Graph<T> minimumSpanningTree(T start) {
		throw new UnsupportedOperationException("MST is not typically defined for directed graphs.");
	}

	@Override
	public boolean addEdge(T v1, T v2, double weight) {
		super.vertices.putIfAbsent(v1, new HashSet<>());
		super.vertices.putIfAbsent(v2, new HashSet<>());
		return super.vertices.get(v1).add(new Edge<>(v1, v2, weight));
	}

	@Override
	public boolean removeEdge(T v1, T v2) {
		return super.vertices.containsKey(v1) && super.vertices.get(v1).removeIf(edge -> edge.DESTINATION.equals(v2));
	}

	@Override
	protected boolean shouldAddEdge(T v1, T v2) {
		return true;
	}
}


















/**
 * The GraphView class provides a GUI to visualize graphs.
 * @author Daniel Tongu
 */
class GraphView extends JFrame {
	private List<Edge<Vertex, Vertex>> path;
	private Graph<Vertex> originalGraph;
	private Graph<Vertex> displayedGraph;
	private PaintManager<Vertex> paintManager;
	private JPanel drawingPanel;

	/**
	 * Constructs a new GraphView instance.
	 */
	public GraphView() {
		super.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		int minSize = 500;
		super.setSize(minSize + (minSize * 3/4), minSize);
		super.setLocationRelativeTo(null);

		JScrollPane scrollPane = new JScrollPane(createDrawingPanel());
		scrollPane.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED);
		scrollPane.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
		super.getContentPane().add(scrollPane, BorderLayout.CENTER);
		((JPanel) super.getContentPane()).setBorder(BorderFactory.createEmptyBorder(0, 5, 15, 5));

		originalGraph = getDefaultGraph();
		displayedGraph = originalGraph;

		setupMenuBar();
	}

	/**
	 * Creates the drawing panel for the graph.
	 * @return the created drawing panel
	 */
	private JPanel createDrawingPanel() {
		int height = getHeight();
		paintManager = new PaintManager<>();
		drawingPanel = new JPanel() {
			@Override
			protected void paintComponent(Graphics g) {
				super.paintComponent(g);
				setBackground(paintManager.getPanelBackgroundColor());
				if (displayedGraph != null) {
					Dimension size = paintManager.calculateGraphPanelSize(displayedGraph, height);
					setPreferredSize(size);
					paintManager.drawGraph((Graphics2D) g, size.width, size.height, displayedGraph, path);
					setTitle("Visualization" + (displayedGraph instanceof UndirectedGraph<?> ? " - Undirected " : " - Directed") + "Graph");
				}
			}
		};
		drawingPanel.setBackground(paintManager.getPanelBackgroundColor());
		return drawingPanel;
	}

	 /**
	 * Creates a default graph if no graph is loaded.
	 * @return the created default graph
	 */
	private Graph<Vertex> getDefaultGraph() {
		Vertex[] vertices = {
				new Vertex("A", 50, 10),
				new Vertex("B", 10, 40),
				new Vertex("C", 50, 60),
				new Vertex("D", 90, 40),
				new Vertex("E", 10, 80),
				new Vertex("F", 50, 110),
				new Vertex("G", 90, 80)
		};

		Graph<Vertex> graph = new UndirectedGraph<>();

		for (Vertex vertex : vertices) {
			graph.addVertex(vertex);
		}

		graph.addEdge(vertices[0], vertices[1], 1);
		graph.addEdge(vertices[0], vertices[2], 1);
		graph.addEdge(vertices[0], vertices[3], 10);
		graph.addEdge(vertices[1], vertices[2], 16);
		graph.addEdge(vertices[2], vertices[3], 8);
		graph.addEdge(vertices[2], vertices[4], 4);
		graph.addEdge(vertices[2], vertices[5], 9);
		graph.addEdge(vertices[2], vertices[6], 14);
		graph.addEdge(vertices[3], vertices[6], 17);
		graph.addEdge(vertices[4], vertices[5], 6);
		graph.addEdge(vertices[5], vertices[6], 2);

		return graph;
	}

	/**
	 * Sets up the menu bar for the application.
	 */
	private void setupMenuBar() {
		JMenuBar menuBar = new JMenuBar();

		// File menu
		JMenu fileMenu = new JMenu("File");
		fileMenu.add(createMenuItem("New", e -> newGraph()));
		fileMenu.add(createMenuItem("Open", e -> openGraph()));
		fileMenu.add(createMenuItem("Save", e -> saveGraph()));

		// Vertex menu
		JMenu vertexMenu = new JMenu("Vertex");
		vertexMenu.add(createMenuItem("Add Vertex", this::addVertex));
		vertexMenu.add(createMenuItem("Delete Vertex", this::deleteVertex));
		vertexMenu.add(createMenuItem("Set Location", this::setVertexLocation));
		vertexMenu.add(createMenuItem("Show Location", this::showVertexLocation));

		// Edge menu
		JMenu edgeMenu = new JMenu("Edge");
		edgeMenu.add(createMenuItem("Add Edge", this::addEdge));
		edgeMenu.add(createMenuItem("Delete Edge", this::deleteEdge));
		edgeMenu.add(createMenuItem("Set Weight", this::setEdgeWeight));
		edgeMenu.add(createMenuItem("Show Weight", this::showEdgeWeight));

		// Traverse menu
		JMenu traverseMenu = new JMenu("Traversal");
		traverseMenu.add(createMenuItem("Clear", this::clearTraversal));
		traverseMenu.add(createMenuItem("Show BFS", this::performBFS));
		traverseMenu.add(createMenuItem("Show DFS",	this::performDFS));
		traverseMenu.add(createMenuItem("Show MST", this::performMST));
		traverseMenu.add(createMenuItem("Show Path", this::findShortestPath));

		JMenu settingsMenu = new JMenu("View Settings");
		settingsMenu.add(createCheckboxMenuItem("Show Dark Mode", this::toggleDarkMode, paintManager.getCurrentTheme() == ColorManager.ColorTheme.DARK));
		settingsMenu.add(createCheckboxMenuItem("Show undirected", this::toggleGraphType, displayedGraph instanceof UndirectedGraph));
		settingsMenu.add(createCheckboxMenuItem("Show Weights", this::toggleShowWeights, false));

		menuBar.setLayout(new FlowLayout(FlowLayout.LEFT));
		menuBar.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

		menuBar.add(fileMenu);
		menuBar.add(vertexMenu);
		menuBar.add(edgeMenu);
		menuBar.add(traverseMenu);
		menuBar.add(settingsMenu);

		setJMenuBar(menuBar);
	}

	/**
	 * Creates a menu item with the specified title and action listener.
	 * @param title the title of the menu item
	 * @param actionListener the action listener for the menu item
	 * @return the created menu item
	 */
	private JMenuItem createMenuItem(String title, ActionListener actionListener) {
		JMenuItem menuItem = new JMenuItem(title);
		menuItem.addActionListener(actionListener);
		return menuItem;
	}

	/**
	 * Creates a JCheckBoxMenuItem with the specified title and action listener.
	 *
	 * @param title          the title of the checkbox menu item
	 * @param actionListener the action listener for the checkbox menu item
	 * @param selected       the initial selected state of the checkbox menu item
	 * @return the created JCheckBoxMenuItem
	 */
	private JCheckBoxMenuItem createCheckboxMenuItem(String title, ActionListener actionListener, boolean selected) {
		JCheckBoxMenuItem checkboxMenuItem = new JCheckBoxMenuItem(title, selected);
		checkboxMenuItem.addActionListener(actionListener);
		return checkboxMenuItem;
	}


	/**
	 * Creates a new graph based on user selection (directed or undirected).
	 */
	private void newGraph() {
		Object[] options = {"Directed", "Undirected"};
		int n = JOptionPane.showOptionDialog(this,
				"Choose the type of graph:",
				"New Graph",
				JOptionPane.YES_NO_CANCEL_OPTION,
				JOptionPane.QUESTION_MESSAGE,
				null,
				options,
				options[1]);

		if (n == 0) {
			displayedGraph = new DirectedGraph<>();
		} else if (n == 1) {
			displayedGraph = new UndirectedGraph<>();
		}
		originalGraph = displayedGraph;
		drawingPanel.repaint();
	}

	/**
	 * Opens an existing graph from a file.
	 */
	private void openGraph() {
		JFileChooser fileChooser = new JFileChooser();
		int returnValue = fileChooser.showOpenDialog(this);
		if (returnValue == JFileChooser.APPROVE_OPTION) {
			File selectedFile = fileChooser.getSelectedFile();
			Graph<Vertex> load = deserializeGraph(selectedFile.getAbsolutePath());
			if (load != null) {
				displayedGraph = load;
				originalGraph = displayedGraph;
				drawingPanel.repaint();
			}
		}
	}

	/**
	 * Saves the current graph to a file.
	 */
	private void saveGraph() {
		JFileChooser fileChooser = new JFileChooser();
		int returnValue = fileChooser.showSaveDialog(this);
		if (returnValue == JFileChooser.APPROVE_OPTION) {
			File selectedFile = fileChooser.getSelectedFile();
			if (serializeGraph(displayedGraph, selectedFile.getAbsolutePath())) {
				originalGraph = displayedGraph;
			}
		}
	}

	/**
	 * Adds a new vertex to the graph.
	 * @param e the action event
	 */
	private void addVertex(ActionEvent e) {
		JPanel panel = new JPanel(new GridLayout(3, 2));
		JTextField labelField = new JTextField(10);
		JTextField xField = new JTextField(10);
		JTextField yField = new JTextField(10);

		panel.add(new JLabel("Label:"));
		panel.add(labelField);
		panel.add(new JLabel("X Coordinate:"));
		panel.add(xField);
		panel.add(new JLabel("Y Coordinate:"));
		panel.add(yField);

		int result = JOptionPane.showConfirmDialog(this, panel, "Add Vertex", JOptionPane.OK_CANCEL_OPTION);
		if (result == JOptionPane.OK_OPTION) {
			try {
				String label = labelField.getText();
				int x = Integer.parseInt(xField.getText());
				int y = Integer.parseInt(yField.getText());
				Vertex newVertex = new Vertex(label, x, y);
				if (displayedGraph.addVertex(newVertex)) {
					drawingPanel.repaint();
					JOptionPane.showMessageDialog(this, "Vertex added successfully.");
				} else {
					JOptionPane.showMessageDialog(this, "Vertex already exists.", "Error", JOptionPane.ERROR_MESSAGE);
				}
			} catch (NumberFormatException ex) {
				JOptionPane.showMessageDialog(this, "Invalid coordinates.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Deletes a vertex from the graph.
	 * @param e the action event
	 */
	private void deleteVertex(ActionEvent e) {
		String vertexLabel = JOptionPane.showInputDialog(this, "Enter the vertex label to delete:");
		if (vertexLabel != null && !vertexLabel.isBlank()) {
			Optional<Vertex> vertex = displayedGraph.getVertices().stream().filter(v -> v.LABEL.equals(vertexLabel)).findFirst();
			if (vertex.isPresent() && displayedGraph.removeVertex(vertex.get())) {
				drawingPanel.repaint();
				JOptionPane.showMessageDialog(this, "Vertex deleted successfully.");
			} else {
				JOptionPane.showMessageDialog(this, "Vertex not found.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Shows the location of a vertex.
	 * @param e the action event
	 */
	private void showVertexLocation(ActionEvent e) {
		String label = JOptionPane.showInputDialog(this, "Enter vertex label:");
		if (label != null && !label.isBlank()) {
			Optional<Vertex> vertex = displayedGraph.getVertices().stream()
					.filter(v -> v.LABEL.equals(label))
					.findFirst();
			if (vertex.isPresent()) {
				JOptionPane.showMessageDialog(this, "Location of " + label + ": (" + vertex.get().x + ", " + vertex.get().y + ")");
			} else {
				JOptionPane.showMessageDialog(this, "Vertex not found.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Sets the location of a vertex.
	 * @param e the action event
	 */
	private void setVertexLocation(ActionEvent e) {
		JPanel panel = new JPanel(new GridLayout(3, 2));
		JTextField labelField = new JTextField(10);
		JTextField xField = new JTextField(10);
		JTextField yField = new JTextField(10);

		panel.add(new JLabel("Vertex Label:"));
		panel.add(labelField);
		panel.add(new JLabel("New X Coordinate:"));
		panel.add(xField);
		panel.add(new JLabel("New Y Coordinate:"));
		panel.add(yField);

		int result = JOptionPane.showConfirmDialog(this, panel, "Edit Vertex Location", JOptionPane.OK_CANCEL_OPTION);
		if (result == JOptionPane.OK_OPTION) {
			try {
				String label = labelField.getText();
				int x = Integer.parseInt(xField.getText());
				int y = Integer.parseInt(yField.getText());
				Optional<Vertex> vertex = displayedGraph.getVertices().stream().filter(v -> v.LABEL.equals(label)).findFirst();
				if (vertex.isPresent()) {
					vertex.get().setLocation(x, y);
					drawingPanel.repaint();
					JOptionPane.showMessageDialog(this, "Vertex location updated.");
				} else {
					JOptionPane.showMessageDialog(this, "Vertex not found.", "Error", JOptionPane.ERROR_MESSAGE);
				}
			} catch (NumberFormatException ex) {
				JOptionPane.showMessageDialog(this, "Invalid coordinates.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Adds an edge between two vertices.
	 * @param e the action event
	 */
	private void addEdge(ActionEvent e) {
		JPanel panel = new JPanel(new GridLayout(3, 2));
		JTextField startField = new JTextField(10);
		JTextField endField = new JTextField(10);
		JTextField weightField = new JTextField(10);

		panel.add(new JLabel("Start Vertex:"));
		panel.add(startField);
		panel.add(new JLabel("End Vertex:"));
		panel.add(endField);
		panel.add(new JLabel("Weight:"));
		panel.add(weightField);

		int result = JOptionPane.showConfirmDialog(this, panel, "Add Edge", JOptionPane.OK_CANCEL_OPTION);
		if (result == JOptionPane.OK_OPTION) {
			try {
				String startLabel = startField.getText();
				String endLabel = endField.getText();
				double weight = Double.parseDouble(weightField.getText());
				Optional<Vertex> startVertex = findVertexByLabel(startLabel);
				Optional<Vertex> endVertex = findVertexByLabel(endLabel);
				if (startVertex.isPresent() && endVertex.isPresent() && displayedGraph.addEdge(startVertex.get(), endVertex.get(), weight)) {
					drawingPanel.repaint();
					JOptionPane.showMessageDialog(this, "Edge added successfully.");
				} else {
					JOptionPane.showMessageDialog(this, "Failed to add edge.", "Error", JOptionPane.ERROR_MESSAGE);
				}
			} catch (NumberFormatException ex) {
				JOptionPane.showMessageDialog(this, "Invalid weight format.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Deletes an edge between two vertices.
	 * @param e the action event
	 */
	private void deleteEdge(ActionEvent e) {
		JPanel panel = new JPanel(new GridLayout(2, 2));
		JTextField startField = new JTextField(10);
		JTextField endField = new JTextField(10);

		panel.add(new JLabel("Start Vertex:"));
		panel.add(startField);
		panel.add(new JLabel("End Vertex:"));
		panel.add(endField);

		int result = JOptionPane.showConfirmDialog(this, panel, "Delete Edge", JOptionPane.OK_CANCEL_OPTION);
		if (result == JOptionPane.OK_OPTION) {
			String startLabel = startField.getText();
			String endLabel = endField.getText();
			Optional<Vertex> startVertex = findVertexByLabel(startLabel);
			Optional<Vertex> endVertex = findVertexByLabel(endLabel);

			if (startVertex.isPresent() && endVertex.isPresent() && displayedGraph.removeEdge(startVertex.get(), endVertex.get())) {
				drawingPanel.repaint();
				JOptionPane.showMessageDialog(this, "Edge deleted successfully.");
			} else {
				JOptionPane.showMessageDialog(this, "Failed to delete edge or edge does not exist.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Sets the weight of an edge between two vertices.
	 * @param e the action event
	 */
	private void setEdgeWeight(ActionEvent e) {
		JPanel panel = new JPanel(new GridLayout(3, 2));
		JTextField v1Field = new JTextField(5);
		JTextField v2Field = new JTextField(5);
		JTextField weightField = new JTextField(5);

		panel.add(new JLabel("First vertex:"));
		panel.add(v1Field);
		panel.add(new JLabel("Second vertex:"));
		panel.add(v2Field);
		panel.add(new JLabel("Weight:"));
		panel.add(weightField);

		int result = JOptionPane.showConfirmDialog(this, panel, "Set Edge Weight", JOptionPane.OK_CANCEL_OPTION);
		if (result == JOptionPane.OK_OPTION) {
			try {
				String v1Label = v1Field.getText();
				String v2Label = v2Field.getText();
				double weight = Double.parseDouble(weightField.getText());
				Optional<Vertex> v1 = findVertexByLabel(v1Label);
				Optional<Vertex> v2 = findVertexByLabel(v2Label);
				if (v1.isPresent() && v2.isPresent() && displayedGraph.removeEdge(v1.get(), v2.get()) && displayedGraph.addEdge(v1.get(), v2.get(), weight)) {
					drawingPanel.repaint();
					JOptionPane.showMessageDialog(this, "Edge weight updated successfully.");
				} else {
					JOptionPane.showMessageDialog(this, "Invalid input or edge does not exist.", "Error", JOptionPane.ERROR_MESSAGE);
				}
			} catch (NumberFormatException ex) {
				JOptionPane.showMessageDialog(this, "Invalid weight format.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Shows the weight of an edge between two vertices.
	 * @param e the action event
	 */
	private void showEdgeWeight(ActionEvent e) {
		JPanel panel = new JPanel(new GridLayout(2, 2));
		JTextField v1Field = new JTextField(5);
		JTextField v2Field = new JTextField(5);

		panel.add(new JLabel("First vertex:"));
		panel.add(v1Field);
		panel.add(new JLabel("Second vertex:"));
		panel.add(v2Field);

		int result = JOptionPane.showConfirmDialog(this, panel, "Show Edge Weight", JOptionPane.OK_CANCEL_OPTION);
		if (result == JOptionPane.OK_OPTION) {
			String v1Label = v1Field.getText();
			String v2Label = v2Field.getText();
			Optional<Vertex> v1 = findVertexByLabel(v1Label);
			Optional<Vertex> v2 = findVertexByLabel(v2Label);
			if (v1.isPresent() && v2.isPresent()) {
				double weight = displayedGraph.getEdgeWeight(v1.get(), v2.get());
				JOptionPane.showMessageDialog(this, "Weight of the edge: " + weight);
			} else {
				JOptionPane.showMessageDialog(this, "Edge not found.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Performs a depth-first search starting from a specified vertex.
	 * @param e the action event
	 */
	private void performDFS(ActionEvent e) {
		String vertexLabel = JOptionPane.showInputDialog(this, "Enter the starting vertex for DFS:");
		Optional<Vertex> vertex = findVertexByLabel(vertexLabel);
		if (vertex.isPresent()) {
			path = displayedGraph.dfs(vertex.get());
			drawingPanel.repaint();
			JOptionPane.showMessageDialog(this, String.format("DFS from %s displayed.", vertexLabel));
		} else {
			JOptionPane.showMessageDialog(this, "Vertex not found.", "Error", JOptionPane.ERROR_MESSAGE);
		}
	}

	/**
	 * Performs a breadth-first search starting from a specified vertex.
	 * @param e the action event
	 */
	private void performBFS(ActionEvent e) {
		String vertexLabel = JOptionPane.showInputDialog(this, "Enter the starting vertex for BFS:");
		Optional<Vertex> vertex = findVertexByLabel(vertexLabel);
		if (vertex.isPresent()) {
			path = displayedGraph.bfs(vertex.get());
			drawingPanel.repaint();
			JOptionPane.showMessageDialog(this, String.format("BFS from %s displayed.", vertexLabel));
		} else {
			JOptionPane.showMessageDialog(this, "Vertex not found.", "Error", JOptionPane.ERROR_MESSAGE);
		}
	}

	/**
	 * Performs a minimum spanning tree starting from a specified vertex.
	 * @param e the action event
	 */
	private void performMST(ActionEvent e) {
		String startVertexLabel = JOptionPane.showInputDialog(this, "Enter start vertex label for MST:");
		if (startVertexLabel != null && !startVertexLabel.isBlank()) {
			try {
				Vertex startVertex = findVertexByLabel(startVertexLabel).orElseThrow(() -> new IllegalArgumentException("Vertex not found."));
				displayedGraph = originalGraph.minimumSpanningTree(startVertex); // Temporarily display the MST
				drawingPanel.repaint();
				JOptionPane.showMessageDialog(this, String.format("MST starting at from %s displayed.", startVertexLabel));
			} catch (IllegalArgumentException | UnsupportedOperationException ex) {
				JOptionPane.showMessageDialog(this, ex.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Finds the shortest path between two vertices.
	 * @param e the action event
	 */
	private void findShortestPath(ActionEvent e) {
		JPanel panel = new JPanel(new GridLayout(2, 2));
		JTextField startField = new JTextField(5);
		JTextField endField = new JTextField(5);

		panel.add(new JLabel("Start vertex:"));
		panel.add(startField);
		panel.add(new JLabel("End vertex:"));
		panel.add(endField);

		int result = JOptionPane.showConfirmDialog(this, panel, "Find Shortest Path", JOptionPane.OK_CANCEL_OPTION);
		if (result == JOptionPane.OK_OPTION) {
			String startLabel = startField.getText();
			String endLabel = endField.getText();
			Optional<Vertex> startVertex = findVertexByLabel(startLabel);
			Optional<Vertex> endVertex = findVertexByLabel(endLabel);

			if (startVertex.isPresent() && endVertex.isPresent()) {
				path = displayedGraph.shortestPath(startVertex.get(), endVertex.get());
				drawingPanel.repaint();
				JOptionPane.showMessageDialog(this, String.format("Shortest path from %s to %s displayed.", startLabel, endLabel));
			} else {
				JOptionPane.showMessageDialog(this, "One or both vertices not found.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/**
	 * Serializes the current graph to a file.
	 * @param graph the graph to serialize
	 * @param filePath the file path to save the graph
	 * @return true if the graph was saved successfully, false otherwise
	 */
	private boolean serializeGraph(Graph<Vertex> graph, String filePath) {
		try (ObjectOutputStream out = new ObjectOutputStream(new FileOutputStream(filePath))) {
			out.writeObject(graph);
			JOptionPane.showMessageDialog(this, "Graph saved successfully!", "Graph Saved", JOptionPane.INFORMATION_MESSAGE);
			return true;
		} catch (IOException ex) {
			JOptionPane.showMessageDialog(this, "Error saving file: " + ex.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
			return false;
		}
	}

	/**
	 * Deserializes a graph from a file.
	 * @param filePath the file path to open the graph
	 * @return the deserialized graph, or null if an error occurred
	 */
	private Graph<Vertex> deserializeGraph(String filePath) {
		try (ObjectInputStream in = new ObjectInputStream(new FileInputStream(filePath))) {
			return (Graph<Vertex>) in.readObject();
		} catch (IOException | ClassNotFoundException ex) {
			JOptionPane.showMessageDialog(this, "Error opening file: " + ex.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
			return null;
		}
	}

	/**
	 * Finds a vertex by its label.
	 * @param label the label of the vertex
	 * @return an optional containing the vertex if found, or empty if not found
	 */
	private Optional<Vertex> findVertexByLabel(String label) {
		return displayedGraph.getVertices().stream().filter(v -> v.LABEL.equals(label)).findFirst();
	}

	/**
	 * Clears the current path and resets the displayed graph to the original graph.
	 */
	private void clearTraversal(ActionEvent e) {
		path = null;
		displayedGraph = originalGraph;
		drawingPanel.repaint();
	}

	/**
	 * Toggles the display of edge weights.
	 */
	private void toggleShowWeights(ActionEvent e) {
		paintManager.showWeights = ((JCheckBoxMenuItem) e.getSource()).isSelected();
		drawingPanel.repaint();
	}

	/**
	 * Toggles between dark and light mode.
	 */
	private void toggleDarkMode(ActionEvent e) {
		paintManager.toggleDarkMode();
		drawingPanel.repaint();
	}

	/**
	 * Toggles the type of the displayed graph between directed and undirected.
	 */
	private void toggleGraphType(ActionEvent e) {
		if (displayedGraph instanceof UndirectedGraph) {
			displayedGraph = convertToDirectedGraph((UndirectedGraph<Vertex>) displayedGraph);
		} else if (displayedGraph instanceof DirectedGraph) {
			displayedGraph = convertToUndirectedGraph((DirectedGraph<Vertex>) displayedGraph);
		}
		drawingPanel.repaint();
	}


	/**
	 * Converts an undirected graph to a directed graph.
	 * @param undirectedGraph the undirected graph
	 * @return the converted directed graph
	 */
	private Graph<Vertex> convertToDirectedGraph(UndirectedGraph<Vertex> undirectedGraph) {
		DirectedGraph<Vertex> directedGraph = new DirectedGraph<>();
		for (Vertex vertex : undirectedGraph.getVertices()) {
			directedGraph.addVertex(vertex);
		}
		for (Edge<Vertex, Vertex> edge : undirectedGraph.getEdges()) {
			directedGraph.addEdge(edge.SOURCE, edge.DESTINATION, edge.weight);
		}
		return directedGraph;
	}

	/**
	 * Converts a directed graph to an undirected graph.
	 * @param directedGraph the directed graph
	 * @return the converted undirected graph
	 */
	private Graph<Vertex> convertToUndirectedGraph(DirectedGraph<Vertex> directedGraph) {
		UndirectedGraph<Vertex> undirectedGraph = new UndirectedGraph<>();
		for (Vertex vertex : directedGraph.getVertices()) {
			undirectedGraph.addVertex(vertex);
		}
		for (Edge<Vertex, Vertex> edge : directedGraph.getEdges()) {
			undirectedGraph.addEdge(edge.SOURCE, edge.DESTINATION, edge.weight);
		}
		return undirectedGraph;
	}

	/**
	 * The main method to run the GraphViewer application.
	 * @param args the command-line arguments
	 */
	public static void main(String[] args) {
		SwingUtilities.invokeLater(() -> {
			GraphView ui = new GraphView();
			ui.setVisible(true);
		});
	}












	/**
	 * The Vertex class represents a vertex in a graph.
	 * @author Daniel Tongu
	 */
	static class Vertex extends Point {
		final String LABEL;

		/**
		 * Constructs a Vertex with the specified label and coordinates.
		 * @param label the label of the vertex
		 * @param x     the x-coordinate of the vertex
		 * @param y     the y-coordinate of the vertex
		 */
		public Vertex(String label, double x, double y) {
			if (label == null || label.isBlank()) {
				throw new IllegalArgumentException("Label cannot be null or blank.");
			}
			this.LABEL = label;
			super.setLocation(x, y);
		}

		@Override
		public boolean equals(Object o) {
			if (this == o) return true;
			if (!(o instanceof Vertex that)) return false;
			return this.LABEL.equals(that.LABEL);
		}

		@Override
		public int hashCode() {
			return LABEL.hashCode();
		}

		@Override
		public String toString() {
			return LABEL;
		}

		/**
		 * Calculates the distance to another vertex.
		 * @param other the other vertex
		 * @return the distance to the other vertex
		 */
		public double distanceTo(Vertex other) {
			return Point.distance(x, y, other.x, other.y);
		}
	}











	/**
	 * A utility class for managing color themes.
	 * @author Daniel Tongu
	 */
	abstract static class ColorManager {
		/**
		 * Enum representing available color themes.
		 */
		public enum ColorTheme {LIGHT, DARK}

		private ColorTheme currentTheme = ColorTheme.DARK; // Default theme;

		// Color constants
		private final Color PANEL_DARK_BACKGROUND_COLOR = new Color(43, 45, 48);
		private final Color PANEL_LIGHT_BACKGROUND_COLOR = Color.WHITE;

		private final Color GRAPH_DARK_COLOR = Color.BLACK;
		private final Color GRAPH_DARK_LABELS_COLOR = new Color(116, 167, 127);
		private final Color GRAPH_DARK_FADE_COLOR = Color.BLACK;

		private final Color GRAPH_LIGHT_COLOR = Color.WHITE;
		private final Color GRAPH_LIGHT_LABELS_COLOR = new Color(235, 80, 57);
		private final Color GRAPH_LIGHT_FADE_COLOR = new Color(234, 236, 238);

		/**
		 * Gets the current color theme.
		 * @return The current color theme.
		 */
		public ColorTheme getCurrentTheme() {
			return currentTheme;
		}

		/**
		 * Gets the background color of the graph panel based on the current theme.
		 * @return The background color.
		 */
		public Color getPanelBackgroundColor() {
			return (currentTheme == ColorTheme.DARK) ? PANEL_DARK_BACKGROUND_COLOR : PANEL_LIGHT_BACKGROUND_COLOR;
		}

		/**
		 * Gets the color of the graph elements based on the current theme.
		 * @return The graph color.
		 */
		public Color getGraphColor() {
			return (currentTheme == ColorTheme.DARK) ? GRAPH_LIGHT_COLOR : GRAPH_DARK_COLOR;
		}

		/**
		 * Gets the fade color of the graph elements based on the current theme.
		 * @return The fade graph color.
		 */
		public Color getGraphFadeColor() {
			return (currentTheme == ColorTheme.DARK) ? GRAPH_DARK_FADE_COLOR : GRAPH_LIGHT_FADE_COLOR;
		}

		/**
		 * Gets the color of the graph labels based on the current theme.
		 * @return The graph labels color.
		 */
		public Color getLabelColor() {
			return (currentTheme == ColorTheme.DARK) ? GRAPH_LIGHT_LABELS_COLOR: GRAPH_DARK_LABELS_COLOR;
		}

		/**
		 * Toggles between dark and light themes.
		 */
		public void toggleDarkMode() {
			currentTheme = (currentTheme == ColorTheme.DARK) ? ColorTheme.LIGHT : ColorTheme.DARK;
		}
	}










	/**
	 * The PaintManager class handles the drawing of the graph.
	 * @param <T> the type of the vertices in the graph.
	 * @author Daniel Tongu
	 */
	static class PaintManager<T extends Vertex> extends ColorManager {
		final int vertexDiameter = 25;
		protected boolean showWeights;

		/**
		 * Draws the graph.
		 * @param g2d         the graphics context
		 * @param panelWidth  the width of the panel
		 * @param panelHeight the height of the panel
		 * @param graph       the graph to draw
		 * @param path        the path to highlight
		 */
		public void drawGraph(Graphics2D g2d, int panelWidth, int panelHeight, Graph<T> graph, List<Edge<T, T>> path) {
			if (graph == null || graph.isEmpty()) return;

			g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
			g2d.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

			// Calculate the graph bounds
			Rectangle graphBounds = calculateGraphBounds(graph);

			// Scale and translate setup
			double scale = calculateScale(graphBounds, panelWidth, panelHeight);
			Point offset = calculateOffset(graphBounds, panelWidth, panelHeight, scale);

			// draw the graph
			drawEdges(g2d, graph, scale, offset, path);
			drawVertices(g2d, graph, scale, offset);
		}

		/**
		 * Draws the vertices of the graph.
		 * @param g2d      the graphics context
		 * @param graph    the graph to draw
		 * @param scale    the scale factor
		 * @param offset   the offset for positioning
		 */
		private void drawVertices(Graphics2D g2d, Graph<T> graph, double scale, Point offset) {
			for (T vertex : graph.getVertices()) {
				drawVertex(g2d, vertex, scale, offset, super.getGraphColor(), super.getLabelColor());
			}
		}

		/**
		 * Draws a single vertex.
		 * @param g2d     the graphics context
		 * @param vertex  the vertex to draw
		 * @param scale   the scale factor
		 * @param offset  the offset for positioning
		 * @param vColor  the color of the vertex
		 * @param lColor  the color of the vertex label
		 */
		private void drawVertex(Graphics2D g2d, Vertex vertex, double scale, Point offset, Color vColor, Color lColor) {
			// Calculate the position of the vertex on the panel
			double x = vertex.x * scale + offset.x - (double) vertexDiameter / 2;
			double y = vertex.y * scale + offset.y - (double) vertexDiameter / 2;

			// Draw the vertex circle
			g2d.setColor(vColor);
			g2d.fill(new Ellipse2D.Double(x, y, vertexDiameter, vertexDiameter));

			// Calculate the font size based on the vertex diameter
			int fontSize = (int) (vertexDiameter / 1.7);
			Font font = new Font("Arial", Font.PLAIN, fontSize);
			g2d.setFont(font);

			// Prepare to draw the label centered
			FontMetrics fm = g2d.getFontMetrics();
			double textWidth = fm.stringWidth(vertex.LABEL);
			double textHeight = fm.getHeight();
			double textX = x + (vertexDiameter - textWidth) / 2;
			double textY = y + (vertexDiameter - textHeight) / 2 + fm.getAscent();

			// Draw the label shadow
			g2d.setColor(Color.BLACK); // Shadow color
			g2d.drawString(vertex.LABEL, (float) (textX + .3), (float) (textY + .3)); // Offset by .3 pixels

			// Draw the label
			g2d.setColor(lColor); // Original label color
			g2d.drawString(vertex.LABEL, (float) textX, (float) textY);
		}

		/**
		 * Draws the edges of the graph.
		 * @param g2d        the graphics context
		 * @param graph      the graph to draw
		 * @param scale      the scale factor
		 * @param offset     the offset for positioning
		 * @param pathEdges  the list of edges in the path
		 */
		private void drawEdges(Graphics2D g2d, Graph<T> graph, double scale, Point offset, List<Edge<T, T>> pathEdges) {
			Set<Edge<T, T>> edges = new HashSet<>(graph.getEdges());

			if (pathEdges != null && !pathEdges.isEmpty()) {
				for (Edge<T, T> edge : pathEdges) {
					drawEdge(g2d, edge, graph.isDirected(), scale, offset, super.getGraphColor());
					edges.remove(edge);
					if (!graph.isDirected()) {
						Edge<T, T> reverseEdge = new Edge<>(edge.DESTINATION, edge.SOURCE, edge.weight);
						edges.remove(reverseEdge);
					}
				}
			}

			Color color = pathEdges != null && !pathEdges.isEmpty() ? super.getGraphFadeColor() : super.getGraphColor();
			for (Edge<T, T> edge : edges) {
				drawEdge(g2d, edge, graph.isDirected(), scale, offset, color);
			}
		}

		/**
		 * Draws a single edge.
		 * @param g2d       the graphics context
		 * @param edge      the edge to draw
		 * @param isDirected whether the graph is directed
		 * @param scale     the scale factor
		 * @param offset    the offset for positioning
		 * @param color     the color of the edge
		 */
		private void drawEdge(Graphics2D g2d, Edge<T, T> edge, boolean isDirected, double scale, Point offset, Color color) {
			T v1 = edge.SOURCE;
			T v2 = edge.DESTINATION;

			g2d.setColor(color);

			int x1 = (int) (v1.x * scale + offset.x);
			int y1 = (int) (v1.y * scale + offset.y);
			int x2 = (int) (v2.x * scale + offset.x);
			int y2 = (int) (v2.y * scale + offset.y);

			g2d.drawLine(x1, y1, x2, y2);

			if (isDirected) {
				drawArrowHead(g2d, x1, y1, x2, y2);
			}

			if (showWeights) {
				String weight = String.format("%.2f", edge.weight);
				int mx = (x1 + x2) / 2;
				int my = (y1 + y2) / 2;
				g2d.drawString(weight, mx, my);
			}
		}

		/**
		 * Draws an arrowhead for a directed edge.
		 * @param g2d the graphics context
		 * @param x1  the x-coordinate of the start point
		 * @param y1  the y-coordinate of the start point
		 * @param x2  the x-coordinate of the end point
		 * @param y2  the y-coordinate of the end point
		 */
		private void drawArrowHead(Graphics2D g2d, int x1, int y1, int x2, int y2) {
			int arrowLength = 23;
			int arrowWidth = 5;

			double angle = Math.atan2(y2 - y1, x2 - x1);

			int xArrowEnd = x2 - (int) (arrowLength * Math.cos(angle));
			int yArrowEnd = y2 - (int) (arrowLength * Math.sin(angle));

			int[] xPoints = {x2, xArrowEnd + (int) (arrowWidth * Math.sin(angle)), xArrowEnd - (int) (arrowWidth * Math.sin(angle))};
			int[] yPoints = {y2, yArrowEnd - (int) (arrowWidth * Math.cos(angle)), yArrowEnd + (int) (arrowWidth * Math.cos(angle))};

			g2d.draw(new Line2D.Double(x1, y1, xArrowEnd, yArrowEnd));
			g2d.fillPolygon(xPoints, yPoints, 3);
		}
		/**
		 * Calculates the scale factor for the graph based on the panel size.
		 * @param graphBounds the bounds of the graph
		 * @param panelWidth  the width of the panel
		 * @param panelHeight the height of the panel
		 * @return the calculated scale factor
		 */
		private double calculateScale(Rectangle graphBounds, int panelWidth, int panelHeight) {
			double graphWidth = graphBounds.getWidth();
			double graphHeight = graphBounds.getHeight();

			// Prevent division by zero or tiny graph dimensions causing infinite scale.
			if (graphWidth == 0) graphWidth = 1;
			if (graphHeight == 0) graphHeight = 1;

			double scaleX = panelWidth / graphWidth;
			double scaleY = panelHeight / graphHeight;

			return Math.min(scaleX, scaleY) * 0.9;
		}

		/**
		 * Calculates the offset to center the graph on the panel.
		 * @param graphBounds the bounds of the graph
		 * @param panelWidth  the width of the panel
		 * @param panelHeight the height of the panel
		 * @param scale       the scale factor
		 * @return the calculated offset point
		 */
		private Point calculateOffset(Rectangle graphBounds, int panelWidth, int panelHeight, double scale) {
			double graphWidth = graphBounds.getWidth() * scale;
			double graphHeight = graphBounds.getHeight() * scale;

			// Calculate the offset to center the graph
			double offsetX = (panelWidth - graphWidth) / 2 - graphBounds.getX() * scale;
			double offsetY = (panelHeight - graphHeight) / 2 - graphBounds.getY() * scale;

			return new Point((int) offsetX, (int) offsetY);
		}

		/**
		 * Calculates the bounding rectangle of the graph.
		 * @param graph the graph to calculate bounds for
		 * @return the bounding rectangle of the graph
		 */
		public Rectangle calculateGraphBounds(Graph<T> graph) {
			double minX = Double.MAX_VALUE, maxX = Double.MIN_VALUE, minY = Double.MAX_VALUE, maxY = Double.MIN_VALUE;

			for (T vertex : graph.getVertices()) {
				if (vertex.x < minX) { minX = vertex.x; }
				if (vertex.x > maxX) { maxX = vertex.x; }
				if (vertex.y < minY) { minY = vertex.y; }
				if (vertex.y > maxY) { maxY = vertex.y; }
			}

			return new Rectangle(
					(int) minX,
					(int) minY,
					(int) (maxX - minX),
					(int) (maxY - minY)
			);
		}

		/**
		 * Calculates the preferred size of the panel to fit the graph.
		 * @param graph   the graph to calculate the panel size for
		 * @param minSize the minimum size of the panel
		 * @return the calculated dimension of the panel
		 */
		public Dimension calculateGraphPanelSize(Graph<T> graph, int minSize) {
			Rectangle bounds = calculateGraphBounds(graph);
			int width = (int) (bounds.getWidth() * 1.1); // Add some padding
			int height = (int) (bounds.getHeight() * 1.1); // Add some padding
			return new Dimension(Math.max(width, minSize), Math.max(height, minSize));
		}
	}

}
