#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>
#include "rapidxml.hpp"
#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include <Windows.h>
#include <errno.h>
#include <cstring>
#include <set>
#include <queue>
#include <cmath>
#include <limits>
#include <chrono>

using namespace std;
using namespace rapidxml;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

struct Node {
    string id;
    double latitude = 0.0;
    double longitude = 0.0;
};

struct Way {
    string id;
    vector<string> nodeRefs;
};

struct AStarNode {
    double g_cost;
    double h_cost;
    double f_cost() const { return g_cost + h_cost; }
    string id;
};

struct DijkstraNode {
    double cost;
    string id;
};

struct visualizeLine {
    float x1, y1, x2, y2;
};

bool operator<(const AStarNode& a, const AStarNode& b) {
    return a.f_cost() > b.f_cost();
}

bool operator<(const DijkstraNode& a, const DijkstraNode& b) {
    return a.cost > b.cost;
}

double heuristic(const Node& a, const Node& b) {
    return sqrt(pow(a.latitude - b.latitude, 2) + pow(a.longitude - b.longitude, 2));
}

unordered_map<string, vector<string>> visitedPathsAStar;
unordered_map<string, vector<string>> visitedPathsPathsDijkstra;
vector<visualizeLine> linesToDraw;




unordered_map<string, vector<pair<string, double>>> adjList;

void createAdjacencyList(const vector<Way>& ways, const unordered_map<string, Node>& nodes) {
    for (const auto& way : ways) {
        for (size_t i = 0; i < way.nodeRefs.size() - 1; i++) {
            string from = way.nodeRefs[i];
            string to = way.nodeRefs[i + 1];

            double cost = heuristic(nodes.at(from), nodes.at(to));
            adjList[from].push_back(make_pair(to, cost));
            adjList[to].push_back(make_pair(from, cost));
        }
    }
    cout << "Adjecency List Size: " << adjList.size() << endl;
    cout << "visitedPathsPathsAStar: " << visitedPathsAStar.size() << endl; 
}

void expand_neighbours_a_star(const string& current_id, priority_queue<AStarNode>& open_set,
    unordered_map<string, double>& g_costs, const string& goal_id,
    const unordered_map<string, Node>& nodes,
    unordered_map<string, string>& came_from,
    unordered_map<string, vector<string>>& visitedPathsAStar) {
    cout << "Expanding neighbours for node: " << current_id << endl;
    for (const auto& neighbour : adjList[current_id]) {
        string neighbour_id = neighbour.first;
        double travel_cost = neighbour.second;

        double tentative_g_cost = g_costs[current_id] + travel_cost;
        /*cout << "tentative costs:" << tentative_g_cost << endl << "g_costs:" << g_costs[current_id] << endl << "neighbour g_cost: " << g_costs[neighbour_id] << endl;*/
        if (tentative_g_cost < g_costs[neighbour_id]) {
            g_costs[neighbour_id] = tentative_g_cost;
            double h_cost = heuristic(nodes.at(neighbour_id), nodes.at(goal_id));
            open_set.push({ tentative_g_cost, h_cost, neighbour_id });
            came_from[neighbour_id] = current_id;
            visitedPathsAStar[current_id].push_back(neighbour_id);
            //cout << "Hinzugefügt zu visitedPathsStar: " << current_id << " -> " << neighbour_id << endl;
        }
    }
}

void expand_neighbours_dijkstra(const string& current_id, priority_queue<DijkstraNode>& open_set,
    unordered_map<string, double>& costs,
    const unordered_map<string, Node>& nodes,
    unordered_map<string, string>& came_from,
    unordered_map<string, vector<string>>& visitedPathsPathsDijkstra) {
    for (const auto& neighbour : adjList[current_id]) {
        string neighbour_id = neighbour.first;
        double travel_cost = neighbour.second;

        double new_cost = costs[current_id] + travel_cost;
        if (new_cost < costs[neighbour_id]) {
            costs[neighbour_id] = new_cost;
            open_set.push({ new_cost, neighbour_id });
            came_from[neighbour_id] = current_id;
            visitedPathsPathsDijkstra[current_id].push_back(neighbour_id);
        }
    }
}
vector<string> a_star_algorithm(const string& start_id, const string& goal_id, const unordered_map<string, Node>& nodes) {
    priority_queue<AStarNode> open_set;
    unordered_map<string, double> g_costs;
    unordered_map<string, string> came_from;
    
    for (const auto& node_pair : nodes) {
        const string& node_id = node_pair.first;
        g_costs[node_id] = (numeric_limits<double>::max)();
    }
    g_costs[start_id] = 0;
    AStarNode start_node{ 0, heuristic(nodes.at(start_id), nodes.at(goal_id)), start_id };
    open_set.push(start_node);
    cout << "Open set size at start of A*: " << open_set.size() << endl;
    while (!open_set.empty()) {
        AStarNode current_node = open_set.top();
        open_set.pop();

        if (current_node.id == goal_id) {
            vector<string> path;
            while (current_node.id != start_id) {
                path.push_back(current_node.id);
                current_node.id = came_from[current_node.id];
            }
            path.push_back(start_id);
            reverse(path.begin(), path.end());
            return path;
        }
        expand_neighbours_a_star(current_node.id, open_set, g_costs, goal_id, nodes, came_from, visitedPathsAStar);
    }

    return {};
}

vector<string> dijkstra_algorithm(const string& start_id, const string& goal_id, const unordered_map<string, Node>& nodes) {
    priority_queue<DijkstraNode> open_set;
    unordered_map<string, double> costs;
    unordered_map<string, string> came_from;

    DijkstraNode start_node{ 0, start_id };
    open_set.push(start_node);
    costs[start_id] = 0;

    while (!open_set.empty()) {
        DijkstraNode current_node = open_set.top();
        open_set.pop();

        if (current_node.id == goal_id) {
            vector<string> path;
            string current_id = goal_id;
            while (current_id != start_id) {
                path.push_back(current_id);
                current_id = came_from[current_id];
            }
            path.push_back(start_id);
            reverse(path.begin(), path.end());
            return path;
        }

        expand_neighbours_dijkstra(current_node.id, open_set, costs, nodes, came_from, visitedPathsPathsDijkstra);
    }
    return {};
}

void addLineToDraw(float x1, float y1, float x2, float y2) {
    linesToDraw.push_back({ x1, y1, x2, y2 });
}


bool isRoad(xml_node<>* way) {
    for (xml_node<>* tag = way->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
        string key = tag->first_attribute("k")->value();
        string value = tag->first_attribute("v")->value();
        if (key == "highway") {
            static const set<string> roadTypes = {
                "motorway", "trunk", "primary", "secondary",
                "tertiary", "unclassified", "residential",
                "service", "motorway_link", "trunk_link",
                "primary_link", "secondary_link", "tertiary_link"
            };
            if (roadTypes.find(value) != roadTypes.end()) {
                return true;
            }
        }
    }
    return false;
}

double minLat, minLon, maxLat, maxLon;
unordered_map<string, Node> nodes;

void parseOSMFile(const string& filename, unordered_map<string, Node>& nodes, vector<Way>& ways,
    double& minLat, double& minLon, double& maxLat, double& maxLon) {
    ifstream file(filename);
    if (!file.is_open()) {
        char errorMsg[256];
        strerror_s(errorMsg, sizeof(errorMsg), errno);
        cerr << "Unable to open file: " << filename << ". Error: " << errorMsg << endl;
        return;
    }

    vector<char> buffer((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    buffer.push_back('\0');

    xml_document<> doc;
    doc.parse<0>(&buffer[0]);

    xml_node<>* bounds = doc.first_node("osm")->first_node("bounds");
    if (bounds) {
        minLat = stod(bounds->first_attribute("minlat")->value());
        minLon = stod(bounds->first_attribute("minlon")->value());
        maxLat = stod(bounds->first_attribute("maxlat")->value());
        maxLon = stod(bounds->first_attribute("maxlon")->value());
    }
    else {
        cerr << "No bounds found in OSM file" << endl;
        return;
    }

    for (xml_node<>* node = doc.first_node("osm")->first_node("node"); node; node = node->next_sibling("node")) {
        Node n;
        n.id = node->first_attribute("id")->value();
        n.latitude = stod(node->first_attribute("lat")->value());
        n.longitude = stod(node->first_attribute("lon")->value());
        nodes[n.id] = n;
    }

    for (xml_node<>* way = doc.first_node("osm")->first_node("way"); way; way = way->next_sibling("way")) {
        if (isRoad(way)) {
            Way w;
            w.id = way->first_attribute("id")->value();
            for (xml_node<>* nd = way->first_node("nd"); nd; nd = nd->next_sibling("nd")) {
                w.nodeRefs.push_back(nd->first_attribute("ref")->value());
            }
            ways.push_back(w);
        }
    }
}

void convertCoordinates(double latitude, double longitude, float& x, float& y) {
    const double MIN_LATITUDE = minLat;
    const double MAX_LATITUDE = maxLat;
    const double MIN_LONGITUDE = minLon;
    const double MAX_LONGITUDE = maxLon;

    x = static_cast<float>((longitude - MIN_LONGITUDE) / (MAX_LONGITUDE - MIN_LONGITUDE) * 2 - 1);
    y = static_cast<float>((latitude - MIN_LATITUDE) / (MAX_LATITUDE - MIN_LATITUDE) * 2 - 1);
}
const GLchar* vertexShaderSource = R"glsl(
    #version 330 core
    layout (location = 0) in vec2 position;
    void main() {
        gl_Position = vec4(position, 0.0, 1.0);
    }
)glsl";

const GLchar* fragmentShaderSource = R"glsl(
    #version 330 core
    out vec4 color;
    uniform vec3 lineColor;
    void main() {
        color = vec4(lineColor, 1.0);
    }
)glsl";

GLuint createShaderProgram() {
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);


    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);



    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);


    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return shaderProgram;
}

GLuint VBO, VAO;

void initRendering() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
}

void drawConnection(float x1, float y1, float x2, float y2, GLuint shaderProgram, float r, float g, float b) {
    float vertices[] = { x1, y1, x2, y2 };

    //cout << "Drawing line from (" << x1 << ", " << y1 << ") to (" << x2 << ", " << y2 << ")/node" << endl;

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glUseProgram(shaderProgram);
    glUniform3f(glGetUniformLocation(shaderProgram, "lineColor"), r, g, b);
    glDrawArrays(GL_LINES, 0, 2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void drawAllLines(GLuint shaderProgram, float r, float g, float b) {
    for (const auto& line : linesToDraw) {
        drawConnection(line.x1, line.y1, line.x2, line.y2, shaderProgram, r, g, b);
    }
}

void drawNode(float x, float y, GLuint shaderProgram) {
    float pointSize = 5.0f; 
    float vertices[] = { x, y }; 

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glPointSize(pointSize);
    glDrawArrays(GL_POINTS, 0, 1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void drawMap(const unordered_map<string, Node>& nodes, const vector<Way>& ways, GLuint shaderProgram) {
    for (const auto& way : ways) {
        for (size_t i = 1; i < way.nodeRefs.size(); i++) {
            string node_id1 = way.nodeRefs[i - 1];
            string node_id2 = way.nodeRefs[i];

            if (nodes.find(node_id1) != nodes.end() && nodes.find(node_id2) != nodes.end()) {
                const Node& node1 = nodes.at(node_id1);
                const Node& node2 = nodes.at(node_id2);

                float x1, y1, x2, y2;
                convertCoordinates(node1.latitude, node1.longitude, x1, y1);
                convertCoordinates(node2.latitude, node2.longitude, x2, y2);
                drawConnection(x1, y1, x2, y2, shaderProgram, 0.5294f, 0.8078f, 0.9804f);
            }
            else {
                cerr << "Invalid node reference in way" << endl;
            }
        }
    }

}

void visualizeVisitedPaths(const unordered_map<string, vector<string>>& visitedPaths,
    const unordered_map<string, Node>& nodes,
    GLuint shaderProgram,
    //float r, float g, float b,
    size_t& currentIndex,
    high_resolution_clock::time_point& lastTime,
    const int delaysInMs) {

    auto currentTime = high_resolution_clock::now();
    auto elapsedTime = duration_cast<milliseconds>(currentTime - lastTime).count();

    /*cout << "Current Index: " << currentIndex << ", Elapsed Time: " << elapsedTime << "ms/node" << endl;*/
    if (elapsedTime > delaysInMs) {
        auto it = visitedPaths.begin();
        std::advance(it, currentIndex);
        if (it != visitedPaths.end()) {
            const Node& fromNode = nodes.at(it->first);
            /*cout << "Drawing path from node: " << it->first << endl;*/
            for (const string& toNodeId : it->second) {
                const Node& toNode = nodes.at(toNodeId);
                float x1, y1, x2, y2;
                convertCoordinates(fromNode.latitude, fromNode.longitude, x1, y1);
                convertCoordinates(toNode.latitude, toNode.longitude, x2, y2);
                //cout << "Drawing connection from (" << x1 << ", " << y1 << ") to (" << x2 << ", " << y2 << ")/node" << endl;
                /*drawConnection(x1, y1, x2, y2, shaderProgram, r, g, b);*/
                addLineToDraw(x1, y1, x2, y2);
            }
            currentIndex++;
            lastTime = currentTime;
        }
    }

    //for (const auto& pair : visitedPaths) {
    //    const Node& fromNode = nodes.at(pair.first);
    //    cout << "von Knoten:" << pair.first << ", Anzahl der Nachbarn: " << pair.second.size() << endl;
    //    for (const string& toNodeId : pair.second) {
    //        const Node& toNode = nodes.at(toNodeId);
    //        
    //        float x1, y1, x2, y2;
    //        convertCoordinates(fromNode.latitude, fromNode.longitude, x1, y1);
    //        convertCoordinates(toNode.latitude, toNode.longitude, x2, y2);
    //        cout << "Zeichne Verbindung von (" << x1 << ", " << y1 << ") nach (" << x2 << ", " << y2 << ")" << endl;
    //        drawConnection(x1, y1, x2, y2, shaderProgram, r, g, b);
    //    }
    //}
}


void initOpenGL(GLFWwindow*& window) {
    if (!glfwInit()) {
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

    window = glfwCreateWindow(800, 600, "Bierstadt Map", NULL, NULL);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);

    if (glewInit() != GLEW_OK) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
}

int main() {
    GLFWwindow* window;
    initOpenGL(window);

   unordered_map<string, Node> nodes;
    vector<Way> ways;

    cout << "Parsing OSM file" << endl;
    parseOSMFile(R"(C:\Users\sleit\source\repos\Pathfinding\Pathfinding\Bierstadt.osm)", nodes, ways, minLat, minLon, maxLat, maxLon);
    
    createAdjacencyList(ways, nodes);

    //string start_id = "20113732";
    string start_id = "266003505";
    if (adjList.find(start_id) != adjList.end()) {
        cout << "Nachbar von " << start_id << ": " << endl;
        for (const auto& neighbour : adjList[start_id]) {
            cout << " - " << neighbour.first << " mit Kosten: " << neighbour.second << endl;

        }
    }
    else {
        cout << "Keine Nachbarn für Knoten: " << start_id << endl;
    }
    string goal_id = "1777211890";
    //string goal_id = "21723528";
    vector<string> pathAStar = a_star_algorithm(start_id, goal_id, nodes);
    vector<string> pathDijkstra = dijkstra_algorithm(start_id, goal_id, nodes);


    bool showAStarPath = false;
    bool showDijkstraPath = false;

    size_t currentIndexAStar = 0;
    size_t currentIndexDijkstra = 0;
    auto lastTimeAStar = high_resolution_clock::now();
    auto lastTimeDijkstra = high_resolution_clock::now();
    const int delayInMs = 0;

    GLuint shaderProgram = createShaderProgram();
    initRendering();
    
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glLineWidth(0.5f);

        drawMap(nodes, ways, shaderProgram);
        visualizeVisitedPaths(visitedPathsAStar, nodes, shaderProgram/*, 0.48627f, 0.9882f, 0.0f*/, currentIndexAStar, lastTimeAStar, delayInMs);
        drawAllLines(shaderProgram, 0.48627f, 0.9882f, 0.0f);
        glDebugMessageCallback;
       /* if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            showAStarPath = !showAStarPath;

        }
        if (showAStarPath) {
           
        }
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            showDijkstraPath = !showDijkstraPath;

        }
        if (showDijkstraPath) {
            visualizeVisitedPaths(visitedPathsPathsDijkstra, nodes, shaderProgram, 0.5568f, 0.9882f, 0.0f);
        }*/

        

        glfwSwapBuffers(window);
    }

    glfwTerminate();
    return 0;
}
