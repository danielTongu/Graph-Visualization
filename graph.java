import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Ellipse2D;
import java.io.*;
import java.util.*;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;


/**
 * The Graph class represents an abstract graph structure.
 * @param <T> the type of the vertices in the graph
 *
 * @author Daniel Tongu
 */
public abstract class Graph<T> implements Serializable {
	protected final Map<T, Set<Edge<T, T>>> vertices = new HashMap<>();

	/**
	 * Returns whether the graph is directed.
	 * @return true if the graph is directed, false otherwise
	 */
	public abstract boolean isDirected();

	/**
	 * Returns the number of vertices in the graph.
	 * @return the number of vertices
	 */
	public int size() {
		return vertices.size();
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
		vertices.values().forEach(edges -> edges.removeIf(edge -> edge.SECOND.equals(vertex)));
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
	 * Returns the set of edges connected to the specified vertex.
	 * @param vertex the vertex
	 * @return the set of edges connected to the vertex
	 */
	public Set<Edge<T, T>> getVertexEdges(T vertex) {
		return vertices.getOrDefault(vertex, Collections.emptySet());
	}

	/**
	 * Adds a weighted edge between the specified vertices with the given weight.
	 * @param v1 the first vertex
	 * @param v2 the second vertex
	 * @param weight the weight of the edge
	 * @return true if the edge was added, false if it already existed
	 */
	public abstract boolean addEdge(T v1, T v2, double weight);

	/**
	 * Adds an edge between the specified vertices with a default weight of zero.
	 * @param v1 the first vertex
	 * @param v2 the second vertex
	 * @return true if the edge was added, false if it already existed
	 */
	public boolean addEdge(T v1, T v2) {
		return addEdge(v1, v2, 0);
	}

	/**
	 * Removes the edge between the specified vertices.
	 * @param v1 the first vertex
	 * @param v2 the second vertex
	 * @return true if the edge was removed, false if it did not exist
	 */
	public abstract boolean removeEdge(T v1, T v2);

	/**
	 * Returns a list of all edges in the graph.
	 * @return a list of all edges
	 */
	public abstract List<Edge<T, T>> getEdges();

	/**
	 * Returns the weight of the edge between the specified vertices.
	 * @param t1 the first vertex
	 * @param t2 the second vertex
	 * @return the weight of the edge, or NaN if no edge exists
	 */
	public double getEdgeWeight(T t1, T t2) {
		if (vertices.containsKey(t1)) {
			for (Edge<T, T> edge : vertices.get(t1)) {
				if (edge.SECOND.equals(t2)) {
					return edge.weight;
				}
			}
		}
		return Double.NaN;
	}
	/**
	 * Performs a depth-first search (DFS) starting from the specified vertex.
	 * @param startVertex the starting vertex
	 * @return a list of edges in the order they were visited
	 */
	public List<Edge<T, T>> dfs(T startVertex) {
		List<Edge<T, T>> dfsList = new ArrayList<>();

		if (vertices.containsKey(startVertex)) {
			Set<T> visited = new HashSet<>();
			Stack<Edge<T, T>> edgeStack = new Stack<>();
			addEdgesToStack(visited, edgeStack, startVertex);

			while (!edgeStack.isEmpty()) {
				Edge<T, T> currentEdge = edgeStack.pop();
				T currentVertex = currentEdge.SECOND;
				if (!visited.contains(currentVertex)) {
					addEdgesToStack(visited, edgeStack, currentVertex);
					dfsList.add(currentEdge);
				}
			}
		}

		return dfsList;
	}

	/**
	 * Adds the edges of the current vertex to the stack and marks the vertex as visited.
	 * @param visited the set of visited vertices
	 * @param edgeStack the stack of edges to be processed
	 * @param currentVertex the current vertex whose edges are to be added to the stack
	 */
	private void addEdgesToStack(Set<T> visited, Stack<Edge<T, T>> edgeStack, T currentVertex) {
		visited.add(currentVertex);
		Comparator<Edge<T, T>> comparator = Comparator.comparingInt(edge -> edge.SECOND.hashCode());
		PriorityQueue<Edge<T, T>> currentVertexEdges = new PriorityQueue<>(comparator.reversed());
		currentVertexEdges.addAll(vertices.get(currentVertex));
		for (Edge<T, T> edge : currentVertexEdges) {
			edgeStack.push(edge);
		}
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
		Queue<T> queue = new PriorityQueue<>(Comparator.comparingInt(Object::hashCode));
		queue.add(startVertex);
		Set<T> seen = new HashSet<>();
		seen.add(startVertex);

		while (!queue.isEmpty()) {
			T currentVertex = queue.poll();
			Queue<Edge<T, T>> currentVertexEdges = new PriorityQueue<>(Comparator.comparingInt(edge -> edge.SECOND.hashCode()));
			currentVertexEdges.addAll(vertices.get(currentVertex));

			for (Edge<T, T> edge : currentVertexEdges) {
				T neighbor = edge.SECOND;
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
				T neighbor = edge.SECOND;
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
		return getVertexEdges(v1).stream().filter(edge -> edge.SECOND.equals(v2)).findFirst().orElse(null);
	}

	/**
	 * Returns the minimum spanning tree of the graph starting from the specified vertex.
	 * @param start the starting vertex
	 * @return the minimum spanning tree
	 */
	public abstract Graph<T> minimumSpanningTree(T start);
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
		boolean removedV1 = super.vertices.containsKey(v1) && super.vertices.get(v1).removeIf(edge -> edge.SECOND.equals(v2));
		boolean removedV2 = super.vertices.containsKey(v2) && super.vertices.get(v2).removeIf(edge -> edge.SECOND.equals(v1));
		return removedV1 && removedV2;
	}

	@Override
	public List<Edge<T, T>> getEdges() {
		List<Edge<T, T>> edges = new ArrayList<>();
		Set<String> seenEdges = new HashSet<>(); // To avoid duplicating edges
		vertices.forEach((v1, connectedEdges) -> connectedEdges.forEach(edge -> {
			String edgeIdentifier = v1.hashCode() <= edge.SECOND.hashCode() ? v1 + ":" + edge.SECOND : edge.SECOND + ":" + v1;
			if (!seenEdges.contains(edgeIdentifier)) {
				edges.add(new Edge<>(v1, edge.SECOND, edge.weight));
				seenEdges.add(edgeIdentifier);
			}
		}));
		return edges;
	}

	@Override
	public double getEdgeWeight(T t1, T t2) {
		double weight = super.getEdgeWeight(t1, t2);
		if (!Double.isNaN(weight)) {
			return weight;
		}
		return super.getEdgeWeight(t2, t1);
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
			if (added.contains(edge.SECOND)) continue;

			mst.addVertex(edge.SECOND);
			mst.addEdge(edge.FIRST, edge.SECOND, edge.weight);
			added.add(edge.SECOND);

			for (Edge<T, T> newEdge : super.vertices.get(edge.SECOND)) {
				if (!added.contains(newEdge.SECOND)) {
					edgeQueue.add(newEdge);
				}
			}
		}

		return mst;
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
	public boolean addEdge(T v1, T v2, double weight) {
		super.vertices.putIfAbsent(v1, new HashSet<>());
		super.vertices.putIfAbsent(v2, new HashSet<>());
		return super.vertices.get(v1).add(new Edge<>(v1, v2, weight));
	}

	@Override
	public boolean removeEdge(T v1, T v2) {
		return super.vertices.containsKey(v1) && super.vertices.get(v1).removeIf(edge -> edge.SECOND.equals(v2));
	}

	@Override
	public List<Edge<T, T>> getEdges() {
		List<Edge<T, T>> edges = new ArrayList<>();
		vertices.forEach((v1, connectedEdges) -> connectedEdges.forEach(edge -> {
			edges.add(new Edge<>(v1, edge.SECOND, edge.weight));
		}));
		return edges;
	}

	@Override
	public Graph<T> minimumSpanningTree(T start) {
		throw new UnsupportedOperationException("MST is not typically defined for directed graphs.");
	}
}






/**
 * The Edge class represents an edge between the specified vertices, and its weight.
 * @param <U> the type of the first vertex
 * @param <V> the type of the second vertex
 *
 * @author Daniel Tongu
 */
final class Edge<U, V> implements Serializable {

	/** the first vertex.*/
	final U FIRST;

	/** the second vertex.*/
	final V SECOND;

	/** the weight of this edge.*/
	double weight;

	/**
	 * Constructs an Edge between the specified vertices, and weight.
	 * @param first  the first vertex
	 * @param second the second vertex
	 * @param weight the weight of the edge
	 */
	public Edge(U first, V second, double weight) {
		this.FIRST = first;
		this.SECOND = second;
		this.weight = weight;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		Edge<?, ?> edge = (Edge<?, ?>) o;
		return Double.compare(edge.weight, weight) == 0 &&
				Objects.equals(FIRST, edge.FIRST) &&
				Objects.equals(SECOND, edge.SECOND);
	}

	@Override
	public int hashCode() {
		return Objects.hash(FIRST, SECOND, weight);
	}

	@Override
	public String toString() {
		return "Edge{" +
				"source=" + FIRST +
				", destination=" + SECOND +
				", weight=" + weight +
				'}';
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
	private JMenu traversalMenu;

	/**
	 * Constructs a new GraphView instance.
	 */
	public GraphView() {
		super.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		int minSize = 350;
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
	 * Creates a menu with the given name and checkbox items.
	 * @param menuName The name of the menu.
	 * @param items Varargs array of checkbox menu item specifications.
	 * @return A populated JMenu object.
	 */
	private JMenu createCheckboxMenu(String menuName, MenuItemSpec... items) {
		JMenu menu = new JMenu(menuName);

		ButtonGroup group = new ButtonGroup();
		for (MenuItemSpec item : items) {
			JCheckBoxMenuItem menuItem = new JCheckBoxMenuItem(item.name, item.initialState);
			menuItem.addActionListener(e -> {
				item.action.actionPerformed(e);
				for (Component comp : menu.getMenuComponents()) {
					if (comp instanceof JCheckBoxMenuItem) {
						((JCheckBoxMenuItem) comp).setSelected(comp == menuItem);
					}
				}
			});
			menu.add(menuItem);
			group.add(menuItem);
		}
		return menu;
	}

	/**
	 * Creates a menu with the given name and regular menu items.
	 * @param menuName The name of the menu.
	 * @param items Varargs array of menu item specifications.
	 * @return A populated JMenu object.
	 */
	private JMenu createMenu(String menuName, MenuItemSpec... items) {
		JMenu menu = new JMenu(menuName);

		for (MenuItemSpec item : items) {
			JMenuItem menuItem = new JMenuItem(item.name);
			menuItem.addActionListener(item.action);
			menu.add(menuItem);
		}
		return menu;
	}

	/**
	 * Helper class to hold menu item specifications.
	 */
	private static class MenuItemSpec {
		String name;
		ActionListener action;
		boolean initialState;

		public MenuItemSpec(String name, ActionListener action) {
			this(name, action, false);
		}

		public MenuItemSpec(String name, ActionListener action, boolean initialState) {
			this.name = name;
			this.action = action;
			this.initialState = initialState;
		}
	}

	/**
	 * Sets up the menu bar for the application.
	 */
	private void setupMenuBar() {
		// File menu
		JMenu fileMenu = createMenu("File",
				new MenuItemSpec("New", e -> newGraph()),
				new MenuItemSpec("Open", e -> openGraph()),
				new MenuItemSpec("Save", e -> saveGraph()));

		// Vertex menu
		JMenu vertexMenu = createMenu("Vertex",
				new MenuItemSpec("Add Vertex", this::addVertex),
				new MenuItemSpec("Delete Vertex", this::deleteVertex),
				new MenuItemSpec("Set Vertex Location", this::setVertexLocation),
				new MenuItemSpec("Show Vertex Location", this::showVertexLocation));

		// Edge menu
		JMenu edgeMenu = createMenu("Edge",
				new MenuItemSpec("Add Edge", this::addEdge),
				new MenuItemSpec("Delete Edge", this::deleteEdge),
				new MenuItemSpec("Set Edge Weight", this::setEdgeWeight),
				new MenuItemSpec("Show Edge Weight", this::showEdgeWeight));

		// Traversal menu
		traversalMenu = createCheckboxMenu("Traversal",
				new MenuItemSpec("None", this::clearTraversal, true),
				new MenuItemSpec("Breadth First Search", this::performBFS),
				new MenuItemSpec("Depth First Search", this::performDFS),
				new MenuItemSpec("Minimum Spanning Tree", this::performMST),
				new MenuItemSpec("Shortest Path", this::findShortestPath));

		// View settings menu
		JMenu settingsMenu = createCheckboxMenu("View Settings",
				new MenuItemSpec("Show Edge Weights", this::toggleShowWeights, false),
				new MenuItemSpec("Show Dark Mode", this::toggleDarkMode, paintManager.getCurrentTheme() == ColorManager.ColorTheme.DARK));

		// The menubar
		JMenuBar menuBar = new JMenuBar();
		menuBar.setLayout(new FlowLayout(FlowLayout.LEFT));
		menuBar.add(fileMenu);
		menuBar.add(vertexMenu);
		menuBar.add(edgeMenu);
		menuBar.add(traversalMenu);
		menuBar.add(settingsMenu);

		super.setJMenuBar(menuBar);
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
		String vertexLabel = JOptionPane.showInputDialog(this, "Enter the vertex label to delete:", "Delete Vertex", JOptionPane.PLAIN_MESSAGE);
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
		String vertexLabel = JOptionPane.showInputDialog(this, "Enter vertex label:", "Show Vertex Location", JOptionPane.PLAIN_MESSAGE);
		if (vertexLabel != null && !vertexLabel.isBlank()) {
			Optional<Vertex> vertex = displayedGraph.getVertices().stream()
					.filter(v -> v.LABEL.equals(vertexLabel))
					.findFirst();
			if (vertex.isPresent()) {
				JOptionPane.showMessageDialog(this, "Location of " + vertexLabel + ": (" + vertex.get().x + ", " + vertex.get().y + ")");
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

		int result = JOptionPane.showConfirmDialog(this, panel, "Set Vertex Location", JOptionPane.OK_CANCEL_OPTION);
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
		JTextField weightField = new JTextField("0", 10);

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
				Vertex x = v1.get();
				Vertex y = v2.get();
				int weight = (int) displayedGraph.getEdgeWeight(x, y);
				JOptionPane.showMessageDialog(this, String.format("Weight of the edge [%s, %s] is %d", x.LABEL, y.LABEL, weight));
			} else {
				JOptionPane.showMessageDialog(this, "Edge not found.", "Error", JOptionPane.ERROR_MESSAGE);
			}
		}
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
	 * Resets the traversal menu to "None" selected.
	 */
	private void resetTraversalToNone(ActionEvent e) {
		if (traversalMenu != null) {
			for (Component comp : traversalMenu.getMenuComponents()) {
				if (comp instanceof JCheckBoxMenuItem menuItem) {
					menuItem.setSelected(menuItem.getText().equalsIgnoreCase("None"));
				}
			}
			clearTraversal(e);
		}
	}

	/**
	 * Temporarily unchecks the menu item associated with the given action event.
	 * @param e the action event
	 */
	private void temporarilyUncheckMenuItem(ActionEvent e) {
		JCheckBoxMenuItem item = (JCheckBoxMenuItem) e.getSource();
		item.setSelected(false);
		item.getParent().repaint();
	}

	/**
	 * Helper method to perform a graph traversal.
	 * @param e the action event
	 * @param traversalType the type of traversal ("DFS" or "BFS")
	 * @param traversalFunction the function to perform the traversal
	 */
	private void performTraversal(ActionEvent e, String traversalType, Function<Vertex, List<Edge<Vertex, Vertex>>> traversalFunction) {
		temporarilyUncheckMenuItem(e);

		SwingUtilities.invokeLater(() -> {
			String vertexLabel = JOptionPane.showInputDialog(
					this,
					String.format("Enter the starting vertex for %s:", traversalType),
					String.format("Display %s", traversalType),
					JOptionPane.PLAIN_MESSAGE
			);
			if (vertexLabel != null && !vertexLabel.isBlank()) {
				Optional<Vertex> vertex = findVertexByLabel(vertexLabel);
				if (vertex.isPresent()) {
					path = traversalFunction.apply(vertex.get());
					drawingPanel.repaint();
					JOptionPane.showMessageDialog(this, String.format("%s from %s displayed.", traversalType, vertexLabel));
					((JCheckBoxMenuItem) e.getSource()).setSelected(true);
				} else {
					JOptionPane.showMessageDialog(this, "Vertex not found.", "Error", JOptionPane.ERROR_MESSAGE);
					resetTraversalToNone(e);
				}
			} else {
				resetTraversalToNone(e);
			}
		});
	}

	/**
	 * Performs a depth-first search starting from a specified vertex.
	 * @param e the action event
	 */
	private void performDFS(ActionEvent e) {
		performTraversal(e, "DFS", displayedGraph::dfs);
	}

	/**
	 * Performs a breadth-first search starting from a specified vertex.
	 * @param e the action event
	 */
	private void performBFS(ActionEvent e) {
		performTraversal(e, "BFS", displayedGraph::bfs);
	}

	/**
	 * Performs a minimum spanning tree starting from a specified vertex.
	 * @param e the action event
	 */
	private void performMST(ActionEvent e) {
		temporarilyUncheckMenuItem(e);

		SwingUtilities.invokeLater(() -> {
			String startVertexLabel = JOptionPane.showInputDialog(
					this,
					"Enter starting vertex for MST:",
					"Display Minimum Spanning Tree",
					JOptionPane.PLAIN_MESSAGE
			);
			if (startVertexLabel != null && !startVertexLabel.isBlank()) {
				try {
					Vertex startVertex = findVertexByLabel(startVertexLabel).orElseThrow(() -> new IllegalArgumentException("Vertex not found."));
					displayedGraph = originalGraph.minimumSpanningTree(startVertex); // Temporarily display the MST
					drawingPanel.repaint();
					JOptionPane.showMessageDialog(this, String.format("MST starting at %s displayed.", startVertexLabel));
					((JCheckBoxMenuItem) e.getSource()).setSelected(true);  // Keep MST checked
				} catch (IllegalArgumentException | UnsupportedOperationException ex) {
					JOptionPane.showMessageDialog(this, ex.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
					resetTraversalToNone(e);
				}
			} else {
				resetTraversalToNone(e);
			}
		});
	}

	/**
	 * Finds the shortest path between two vertices.
	 * @param e the action event
	 */
	private void findShortestPath(ActionEvent e) {
		temporarilyUncheckMenuItem(e);

		SwingUtilities.invokeLater(() -> {
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
					if (path.isEmpty()) {
						JOptionPane.showMessageDialog(this, String.format("No path from %s to %s.", startLabel, endLabel));
						resetTraversalToNone(e);
					} else {
						drawingPanel.repaint();
						JOptionPane.showMessageDialog(this, String.format("Shortest path from %s to %s displayed.", startLabel, endLabel));
						((JCheckBoxMenuItem) e.getSource()).setSelected(true);  // Keep Shortest Path checked
					}
				} else {
					JOptionPane.showMessageDialog(this, "One or both vertices not found.", "Error", JOptionPane.ERROR_MESSAGE);
					resetTraversalToNone(e);
				}
			} else {
				resetTraversalToNone(e);
			}
		});
	}

	/**
	 * Finds a vertex by its label.
	 * @param label the label of the vertex
	 * @return an optional containing the vertex if found, or empty if not found
	 */
	private Optional<Vertex> findVertexByLabel(String label) {
		return displayedGraph.getVertices().stream().filter(v -> v.LABEL.equalsIgnoreCase(label)).findFirst();
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
	 * Creates a default graph if no graph is loaded.
	 * @return the created default graph
	 */
	private Graph<Vertex> getDefaultGraph() {
		Random random = new Random();
		int randomNumber = random.nextInt(100); // Generates a random number between 0 and 10
		return (randomNumber % 2 != 0) ? getDefaultUndirectedGraph() : getDefaultDirectedGraph();
	}

	/**
	 * Creates a default undirected graph if no graph is loaded.
	 * @return the created default undirected graph
	 */
	private Graph<Vertex> getDefaultUndirectedGraph() {
		Map<Character, Vertex> vertexMap = new HashMap<>();

		vertexMap.put('A', new Vertex("A", 50, 10));
		vertexMap.put('B', new Vertex("B", 10, 30));
		vertexMap.put('C', new Vertex("C", 50, 50));
		vertexMap.put('D', new Vertex("D", 90, 30));
		vertexMap.put('E', new Vertex("E", 10, 70));
		vertexMap.put('F', new Vertex("F", 50, 95));
		vertexMap.put('G', new Vertex("G", 90, 70));

		Graph<Vertex> graph = new UndirectedGraph<>();

		for (Vertex vertex : vertexMap.values()) {
			graph.addVertex(vertex);
		}

		graph.addEdge(vertexMap.get('A'), vertexMap.get('B'), 1);
		graph.addEdge(vertexMap.get('A'), vertexMap.get('C'), 20);
		graph.addEdge(vertexMap.get('A'), vertexMap.get('D'), 10);
		graph.addEdge(vertexMap.get('B'), vertexMap.get('C'), 16);
		graph.addEdge(vertexMap.get('C'), vertexMap.get('D'), 8);
		graph.addEdge(vertexMap.get('C'), vertexMap.get('E'), 4);
		graph.addEdge(vertexMap.get('C'), vertexMap.get('F'), 9);
		graph.addEdge(vertexMap.get('C'), vertexMap.get('G'), 14);
		graph.addEdge(vertexMap.get('D'), vertexMap.get('G'), 17);
		graph.addEdge(vertexMap.get('E'), vertexMap.get('F'), 6);
		graph.addEdge(vertexMap.get('F'), vertexMap.get('G'), 2);

		return graph;
	}

	/**
	 * Creates a default directed graph if no graph is loaded.
	 * @return the created default directed graph
	 */
	private Graph<Vertex> getDefaultDirectedGraph() {
		Map<Character, Vertex> vertexMap = new HashMap<>();

		vertexMap.put('A', new Vertex("A", 0, 15));
		vertexMap.put('B', new Vertex("B", 20, 5));
		vertexMap.put('C', new Vertex("C", 40, 5));
		vertexMap.put('D', new Vertex("D", 20, 25));
		vertexMap.put('E', new Vertex("E", 40, 25));

		Graph<Vertex> graph = new DirectedGraph<>();

		for (Vertex vertex : vertexMap.values()) {
			graph.addVertex(vertex);
		}

		graph.addEdge(vertexMap.get('A'), vertexMap.get('B'));
		graph.addEdge(vertexMap.get('A'), vertexMap.get('D'));
		graph.addEdge(vertexMap.get('B'), vertexMap.get('C'));
		graph.addEdge(vertexMap.get('B'), vertexMap.get('E'));
		graph.addEdge(vertexMap.get('C'), vertexMap.get('E'));
		graph.addEdge(vertexMap.get('D'), vertexMap.get('B'));
		graph.addEdge(vertexMap.get('D'), vertexMap.get('E'));

		return graph;
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
		final int vertexDiameter = 30;
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
			if (graph == null || graph.size() <= 0) return;

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
			Set<Edge<T, T>> pathEdgesCopy = pathEdges != null ? new HashSet<>(pathEdges) : new HashSet<>();
			boolean drawPath = pathEdges != null && !pathEdges.isEmpty();
			Color color = drawPath ? super.getGraphFadeColor() : super.getGraphColor();

			// First, draw all edges of the graph
			for (Edge<T, T> edge : graph.getEdges()) {
				Edge<T, T> reverseEdge = new Edge<>(edge.SECOND, edge.FIRST, edge.weight);
				if (!pathEdgesCopy.contains(edge) && !pathEdgesCopy.contains(reverseEdge)) {
					drawEdge(g2d, edge, graph.isDirected(), scale, offset, color);
				}
			}

			// Then, draw the path edges
			if (drawPath) {
				color = super.getGraphColor();
				for (Edge<T, T> edge : pathEdges) {
					drawEdge(g2d, edge, graph.isDirected(), scale, offset, color);
				}
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
			T v1 = edge.FIRST;
			T v2 = edge.SECOND;

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
				String weight = String.format("%d",(int) edge.weight);
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
			int arrowLength = 9;
			int arrowWidth = 5;
			int vertexRadius = vertexDiameter / 2;

			// Calculate the angle of the line
			double angle = Math.atan2(y2 - y1, x2 - x1);

			// Calculate the new endpoint of the arrow, which is outside the vertex
			int xArrowEnd = x2 - (int) (vertexRadius * Math.cos(angle));
			int yArrowEnd = y2 - (int) (vertexRadius * Math.sin(angle));

			// Calculate the points for the chevron arrowhead
			int x1Point = xArrowEnd - (int) (arrowLength * Math.cos(angle) - arrowWidth * Math.sin(angle));
			int y1Point = yArrowEnd - (int) (arrowLength * Math.sin(angle) + arrowWidth * Math.cos(angle));
			int x2Point = xArrowEnd - (int) (arrowLength * Math.cos(angle) + arrowWidth * Math.sin(angle));
			int y2Point = yArrowEnd - (int) (arrowLength * Math.sin(angle) - arrowWidth * Math.cos(angle));

			// Draw the arrow shaft from the start to the new end point
			g2d.drawLine(x1, y1, xArrowEnd, yArrowEnd);

			// Draw the chevron arrowhead
			g2d.drawLine(xArrowEnd, yArrowEnd, x1Point, y1Point);
			g2d.drawLine(xArrowEnd, yArrowEnd, x2Point, y2Point);
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
