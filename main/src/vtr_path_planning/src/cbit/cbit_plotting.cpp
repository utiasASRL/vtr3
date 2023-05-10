#include "vtr_path_planning/cbit/cbit_plotting.hpp"


void initialize_plot()
{
      // Testing plots
    matplotlibcpp::figure_size(700, 1400); // Larger more useful size for debugging
    //matplotlibcpp::figure_size(350, 700); // For mini more practical version for everyday use
    matplotlibcpp::ylim(-2, 7);
    matplotlibcpp::xlim(-3, 3);
    matplotlibcpp::title("Curvilinear Batch Informed Trees (C++)");
    matplotlibcpp::xlabel("q [m]");
    matplotlibcpp::ylabel("p [m]");

    // TODO: Configure Title, Legend

    //matplotlibcpp::axis("equal"); // Didnt like the axis equal behaviour
    matplotlibcpp::draw();
    matplotlibcpp::pause(0.001);
    return;
}

// Function for plotting all the edges in the current tree (Called at the conclusion of each batch)
// Note this will also get triggered following state updates and repairs as well
void plot_tree(Tree tree, Node robot_pq, std::vector<double> path_p, std::vector<double> path_q, std::vector<std::shared_ptr<Node>> samples)
{
    // Clear the figure
    matplotlibcpp::clf();

    // Configure plotting keyword settings
    std::map<std::string, std::string> keywords_robot;
    keywords_robot["color"] = "lime";


    // Set sliding window based on current robot state
    matplotlibcpp::ylim((robot_pq.p - 1), (robot_pq.p + 11));
    matplotlibcpp::xlim(-3,3); // TODO: replace with param


    // Iterate through the current tree, plot each edge
    for (int i = 0; i < tree.E.size(); i++)
    {
        std::shared_ptr<Node> vm = std::get<0>(tree.E[i]);
        std::shared_ptr<Node> xm = std::get<1>(tree.E[i]);
        std::vector<double> plot_p_edge = {vm->p,xm->p};
        std::vector<double> plot_q_edge = {-1*vm->q,-1*xm->q};
        matplotlibcpp::plot(plot_q_edge, plot_p_edge, "c");
    }
    // Need one stand in point to populate the legend
    std::vector<double> dummy_edge = {0};
    matplotlibcpp::named_plot("Edges", dummy_edge, dummy_edge, "c");


    // Plot the current path solution
    // Invert the q values
    for (int j= 0; j < path_q.size(); j++)
    {
        path_q[j] = path_q[j] * -1;
    }
    matplotlibcpp::named_plot("BIT* Path", path_q, path_p, "b");


    // Plot a hashed centre line for the taught path
    std::vector<double> taught_p = {0,(*tree.V[0]).p};
    std::vector<double> taught_q = {0,0};
    matplotlibcpp::named_plot("Taught Path", taught_q, taught_p, "r--");
    

    // Plot samples (optional, but seems like compute is okay so why not)
    // I find its actually abit distracting having the samples and it doesnt really help very much so commenting out for now
    /*
    for (int k= 0; k < samples.size(); k++)
    {
        Node sample = *(samples[k]);
        std::vector<double> plot_p_sample= {sample.p,sample.p};
        std::vector<double> plot_q_sample = {-1*sample.q,-1*sample.q};
        matplotlibcpp::plot(plot_q_sample, plot_p_sample, ".k");
    }
    */

    // Plot the current robot state
    matplotlibcpp::named_plot("Robot State", std::vector<double> {-1*robot_pq.q}, std::vector<double> {robot_pq.p}, ".g");
    matplotlibcpp::scatter(std::vector<double> {-1*robot_pq.q}, std::vector<double> {robot_pq.p}, 200.0, keywords_robot); // try to change colour, need to implement


    // Format Axis/title
    matplotlibcpp::title("Batch Informed Trees");
    matplotlibcpp::xlabel("q [m]");
    matplotlibcpp::ylabel("p [m]");
    matplotlibcpp::legend();


    // Display and show while continuing script (adds 1ms delay to draw)
    //matplotlibcpp::draw();
    matplotlibcpp::pause(0.001);
    return;
}

// TODO: Experiment with a Euclidean version that I can display everything in. See whether I can discretize and convert all edges
//       of the tree to euclidean space.

// TODO: Test other plotting functions which update more regularly in places that are more relevant (like robot state updates, published path)
//       I think you can do this, but the problem is clearing the figure, if the plots are asychronous this will be an issue.

void plot_robot(Node robot_pq)
{
    // Configure plotting keyword settings
    std::map<std::string, std::string> keywords_robot;
    keywords_robot["color"] = "lime";

    // Plot the current robot state
    matplotlibcpp::scatter(std::vector<double> {-1*robot_pq.q}, std::vector<double> {robot_pq.p}, 100.0, keywords_robot); // try to change colour, need to implement

    matplotlibcpp::draw();
    matplotlibcpp::pause(0.001);
    return;
}