// Contribution by: Abe Tusk https://github.com/abetusk
// To compile:
// gcc -Wall -Weverything -Wno-float-equal src/examples/simple.c -Isrc -o simple
//
// About:
//
// This example outputs 10 random 2D coordinates, and all the generated edges,
// to standard output. Note that the edges have duplicates, but you can easily
// filter them out.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define JC_VORONOI_IMPLEMENTATION
// If you wish to use doubles
//#define JCV_REAL_TYPE double
//#define JCV_FABS fabs
//#define JCV_ATAN2 atan2
//#define JCV_CEIL ceil
//#define JCV_FLOOR floor
//#define JCV_FLT_MAX 1.7976931348623157E+308

#include "../../../plotting/include/gnuplot-iostream.h"
#include "../include/jc_voronoi.h"

int main() {
  constexpr int NPOINT = 20;

  float bounding_box_min_x = 0.0f;
  float bounding_box_max_x = 1.0f;
  float bounding_box_min_y = 0.0f;
  float bounding_box_max_y = 1.0f;

  jcv_rect bounding_box = {{bounding_box_min_x, bounding_box_min_y},
                           {bounding_box_max_x, bounding_box_max_y}};
  jcv_diagram diagram;
  jcv_point points[NPOINT];
  const jcv_site* sites;
  jcv_graphedge* graph_edge;

  memset(&diagram, 0, sizeof(jcv_diagram));

  srand(0);
  for (int i = 0; i < NPOINT; i++) {
    points[i].x = (float)(rand() / (1.0f + RAND_MAX));
    points[i].y = (float)(rand() / (1.0f + RAND_MAX));
  }

  printf("# Seed sites\n");
  for (int i = 0; i < NPOINT; i++) {
    printf("%f %f\n", (double)points[i].x, (double)points[i].y);
  }

  jcv_diagram_generate(NPOINT, (const jcv_point*)points, &bounding_box, 0,
                       &diagram);
  sites = jcv_diagram_get_sites(&diagram);

  std::string str_plot;

  for (int i = 0; i < diagram.numsites; i++) {
    graph_edge = sites[i].edges;
    int index = 0;
    double first_plot_x;
    double first_plot_y;

    str_plot += "set object " + std::to_string(i + 1) + " polygon from";

    while (graph_edge) {
      if (++index == 1) {
        first_plot_x = (double)graph_edge->pos[0].x;
        first_plot_y = (double)graph_edge->pos[0].y;
      }
      str_plot += " " + std::to_string(graph_edge->pos[0].x) + ", " +
                  std::to_string(graph_edge->pos[0].y) + " to";
      graph_edge = graph_edge->next;
    }
    str_plot += " " + std::to_string(first_plot_x) + ", " +
                std::to_string(first_plot_y) +
                " fc rgb 'red' fillstyle solid " +
                std::to_string((1.0 / diagram.numsites) * i) + " noborder\n ";
  }

  jcv_diagram_free(&diagram);

  Gnuplot gp;
  gp << "reset\n";
  gp << "set terminal x11 size 800, 800 0\n";
  gp << "set xrange [" << std::to_string(bounding_box_min_x) << ":"
     << std::to_string(bounding_box_max_x) << "]\n";
  gp << "set yrange [" << std::to_string(bounding_box_min_y) << ":"
     << std::to_string(bounding_box_max_y) << "]\n";
  gp << "set size ratio -1\n";
  gp << str_plot;

  std::vector<std::pair<double, double>> xy_pts_A;
  for (int i = 0; i < NPOINT; i++) {
    xy_pts_A.push_back(std::make_pair(points[i].x, points[i].y));
  }
  gp << "plot " << gp.file1d(xy_pts_A)
     << " with points lc rgb 'black' notitle\n";
}
