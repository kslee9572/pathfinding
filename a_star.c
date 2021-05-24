#include "a_star.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "util.h"

/******* Helper Functions *********/
/* null_terminate : when malloc is unable to allocate space, terminate
 *
 */
void null_terminate() {
    fprintf(stderr, "out of space\n");
    exit(1);
}

/* cons: create a new linked list node
 *
 * val: value of the element of node
 * next: pointer to next node
 *
 * Returns: a linked list node
 */
intlist_t *cons(int val, intlist_t *next) {
    intlist_t *new = (intlist_t *)malloc(sizeof(intlist_t));

    if (new == NULL)
        null_terminate();

    new->num = val;
    new->next = next;
    return new;
}

/* insert: insert a linked list node to the end of given list
 *
 * list: given linked list
 * val: value of node to be inserted
 * Returns: a graph
 */
intlist_t *insert(intlist_t *list, int val) {
    intlist_t *last = list;
    if (list == NULL)
        return cons(val, NULL);

    while (last->next != NULL) {
        last = last->next;
    }
    last->next = cons(val, NULL);
    return list;
}

/* free_list: frees linked list
 *
 * list: given linked list
 */
void free_list(intlist_t *list) {
    if (list == NULL)
        return;

    free_list(list->next);
    free(list);
}

/* print_list: (for debugging neighbors) prints linked list
 *
 * list: given linked list
 */
void print_list(intlist_t *list) {
    while (list != NULL) {
        printf("neighbor is : %d\n", list->num);
        list = list->next;
    }
}

/* calculate_distnace: calculates Euclidean distance between two nodes in graph
 *
 * graph: the graph
 * num1: node number of first node
 * num2: node number of second node
 *
 * Returns: the Euclidean distance between the two nodes
 */
double calculate_distance(graph_t *graph, int num1, int num2) {
    double y = graph->nodes[num1]->latitude - graph->nodes[num2]->latitude;
    double x = graph->nodes[num1]->longitude - graph->nodes[num2]->longitude;

    return sqrt(pow(x, 2) + pow(y, 2));
}

/* calculate_g_cost: calculates g_cost of target node
 *
 * graph: the graph
 * curr: the node number of current node in the loop
 * target: the node number of target node of which the g_cost will be calculated
 *
 * Returns: the g_cost of target node
 */
double calculate_g_cost(graph_t *graph, int curr, int target) {
    return graph->nodes[curr]->g_cost + calculate_distance(graph, curr, target);
}

/* fcost:  returns fcost (for convenience)
 *
 * graph: the graph
 * curr: the node number of current node
 */
double fcost(graph_t *graph, int curr) {
    return graph->nodes[curr]->f_cost;
}

/* gcost:  returns gcost (for convenience)
 *
 * graph: the graph
 * curr: the node number of current node
 */
double gcost(graph_t *graph, int curr) {
    return graph->nodes[curr]->g_cost;
}

/* set_h_cost: since h_cost is invariant for all nodes, fix h_cost of given node
 *
 * graph: the graph
 * curr: the node number of current node
 * end: the node number of destination node
 */
void set_h_cost(graph_t *graph, int curr, int end) {
    graph->nodes[curr]->h_cost = calculate_distance(graph, curr, end);
}

/* set_f_cost:  fix f_cost once g_cost and h_cost are fixed
 *
 * graph: the graph
 * curr: the node number of current node
 */
void set_f_cost(graph_t *graph, int curr) {
    graph->nodes[curr]->f_cost = graph->nodes[curr]->g_cost +
                                 graph->nodes[curr]->h_cost;
}
/********* GRAPH *********/

/* graph_create: create a graph
 *
 * Returns: a graph
 */
graph_t *graph_create(int num_nodes) {
    graph_t *new = (graph_t *)malloc(sizeof(graph_t));
    node_t **new_nodes = (node_t **)calloc(num_nodes, sizeof(node_t *));

    if (new == NULL || new_nodes == NULL)
        null_terminate();

    new->num_nodes = num_nodes;
    new->nodes = new_nodes;
    return new;
}

/* node_create: create a graph node
 *
 * graph: the graph
 * node_num: node number
 * city_name: the city
 * latitude: city latitude
 * longitude: city longitude
 * 
 */
void node_create(graph_t *graph, int node_num, char *city_name,
                 double latitude, double longitude) {
    node_t *new = (node_t *)malloc(sizeof(node_t));

    if (new == NULL)
        null_terminate();

    new->node_num = node_num;
    new->city_name = city_name;
    new->latitude = latitude;
    new->longitude = longitude;

    new->neighbors = NULL;

    new->parent = NULL;
    new->f_cost = 0;
    new->g_cost = 0;
    new->h_cost = 0;

    graph->nodes[node_num] = new;
}

/* add_edge: add an edge between two nodes
 *
 * graph: the graph
 * node_num1: the first node
 * node_num2: the second node
 * 
 */
void add_edge(graph_t *graph, int node_num1, int node_num2) {
    graph->nodes[node_num1]->neighbors = insert(
        (graph->nodes[node_num1]->neighbors), node_num2);
    graph->nodes[node_num2]->neighbors = insert(
        (graph->nodes[node_num2]->neighbors), node_num1);
}

/* graph_free: free a graph and its nodes
 *
 * graph: the graph
 */
void graph_free(graph_t *graph) {
    if (graph == NULL)
        return;

    for (int i = 0; i < graph->num_nodes; i++) {
        if (graph->nodes[i] == NULL) {
            free(graph->nodes[i]);
            continue;
        }
        free_list(graph->nodes[i]->neighbors);
        free(graph->nodes[i]);
    }
    free(graph);
}

/********* A* SEARCH *********/

/* a_star: performs A* search
 *
 * graph: the graph
 * start_node_num: the staring node number
 * end_node_num: the ending node number
 * 
 * Returns: the distance of the path between the start node and end node
 */
double a_star(graph_t *graph, int start_node_num, int end_node_num) {
    //create open and closed set
    queue_t *open_set = queue_create();
    set_t *closed_set = set_create();

    //initialize
    set_h_cost(graph, start_node_num, end_node_num);
    set_f_cost(graph, start_node_num);
    queue_add(open_set, start_node_num, fcost(graph, start_node_num));

    while (!queue_is_empty(open_set)) {
        int curr = queue_remove(open_set);

        if (curr == end_node_num)
            break;

        intlist_t *curr_list = graph->nodes[curr]->neighbors;
        int neighbor;
        double potential_g;
        //neighbor list
        while (curr_list != NULL) {
            neighbor = curr_list->num;
            set_h_cost(graph, neighbor, end_node_num);
            potential_g = calculate_g_cost(graph, curr, neighbor);

            curr_list = curr_list->next;

            //skip condition1
            if (queue_query(open_set, neighbor)) {
                if (gcost(graph, neighbor) <= potential_g)
                    continue;
            }
            //skip condition2
            if (set_query(closed_set, neighbor))
                continue;

            graph->nodes[neighbor]->g_cost = potential_g;
            set_f_cost(graph, neighbor);
            graph->nodes[neighbor]->parent = graph->nodes[curr];

            if (!queue_query(open_set, neighbor))
                queue_add(open_set, neighbor, fcost(graph, neighbor));
            else
                queue_change_priority(open_set, neighbor, fcost(graph, neighbor));
        }
        set_add(closed_set, curr);
    }

    if (gcost(graph, end_node_num) == 0)
        return -1;
    else
        return fcost(graph, end_node_num);
}