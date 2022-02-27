/*
	CSC D84 - Unit 2 - MiniMax search and adversarial games

	This file contains stubs for implementing a MiniMax search
        procedure with alpha-beta pruning. Please read the assignment
	handout carefully - it describes the game, the data you will
	have to handle, and the search functions you must provide.

	Once you have read the handout carefully, implement your search
	code in the sections below marked with

	**************
	*** TO DO:
	**************

	Make sure to add it to your report.txt file - it will be marked!

	Have fun!

	DO NOT FORGET TO 'valgrind' YOUR CODE - We will check for pointer
	management being done properly, and for memory leaks.

	Starter code: F.J.E. Sep. 15
*/

#include "MiniMax_search.h"

double MiniMax(double gr[graph_size][4], int path[1][2], double minmax_cost[size_X][size_Y], int cat_loc[10][2], int cats, int cheese_loc[10][2], int cheeses, int mouse_loc[1][2], int mode, double (*utility)(int cat_loc[10][2], int cheese_loc[10][2], int mouse_loc[1][2], int cats, int cheeses, int depth, double gr[graph_size][4]), int agentId, int depth, int maxDepth, double alpha, double beta)
{
 /*
   This function is the interface between your solution for the assignment and the driver code. The driver code
   in MiniMax_search_core_GL will call this function once per frame, and provide the following data
   
   Board and game layout:

	Exactly the same as for Assignment 1 - have a look at your code if you need a reminder of how the adjacency
	list and agent positions are stored.	

	Note that in this case, the path will contain a single move - at the top level, this function will provide
	an agent with the 'optimal' mini-max move given the game state.


   IMPORTANT NOTE: Mini-max is a recursive procedure. This function will need to fill-in the mini-max values for 
	 	   all game states down to the maximum search depth specified by the user. In order to do that,
		   the function needs to be called with the correct state at each specific node in the mini-max
		   search tree.

		   The game state is composed of:

			* Mouse, cat, and cheese positions (and number of cats and cheeses)
			
		   At the top level (when this function is called by the mini-max driver code), the game state
		   correspond to the current situation of the game. But once you start recursively calling
		   this function for lower levels of the search tree the positions of agents will have changed.
		   
		   Therefore, you will need to define local variables to keep the game state at each node of the
		   mini-max search tree, and you will need to update this state when calling recursively so that
		   the search does the right thing.

		   This function *must check* whether:
			* A candidate move results in a terminal configuration (cat eats mouse, mouse eats cheese)
			  at which point it calls the utility function to get a value
	 		* Maximum search depth has been reached (depth==maxDepth), at which point it will also call
			  the utility function to get a value
			* Otherwise, call recursively using the candidate configuration to find out what happens
			  deeper into the mini-max tree.

   Arguments:
		gr[graph_size][4]   		- This is an adjacency list for the maze
		path[1][2] 			- Your function will return the optimal mini-max move in this array.
		minmax_cost[size_X][size_Y]	- An array in which your code will store the
						  minimax value for maze locations expanded by
						  the search *when called for the mouse, not
						  for the cats!*

						  This array will be used to provide a visual 
						  display of minimax values during the game.

		cat_loc[10][2], cats   - Location of cats and number of cats (we can have at most 10,
					 but there can be fewer). Only valid cat locations are 0 to (cats-1)
		cheese_loc[10][2], cheeses - Location and number of cheese chunks (again at most 10,
					     but possibly fewer). Valid locations are 0 to (cheeses-1)
		mouse_loc[1][2] - Mouse location - there can be only one!
		mode - Search mode selection:
					mode = 0 	- No alpha-beta pruning
					mode = 1	- Alpha-beta pruning

		(*utility)(int cat_loc[10][2], int cheese_loc[10][2], int mouse_loc[1][2], int cats, int cheeses, int depth, double gr[graph_size][4]);
				- This is a pointer to the utility function which returns a value for a specific game configuration

				   NOTE: Unlike the search assignment, this utility function also gets access to the graph so you can do any processing 					 that requires knowledge of the maze for computing the utility values.

				  * How to call the utility function from within this function : *
					- Like any other function:
						u = utility(cat_loc, cheese_loc, mouse_loc, cats, cheeses, depth, gr);
						
		agentId: Identifies which agent we are doing MiniMax for. agentId=0 for the mouse, agentId in [1, cats] for cats. Notice that recursive calls
                         to this function should increase the agentId to reflect the fact that the next level down corresponds to the next agent! For a game
                         with two cats and a mouse, the agentIds for the recursion should look like 0, 1, 2, 0, 1, 2, ...
	
		depth: Current search depth - whether this is a MIN or a MAX node depends both on depth and agentId.
		
		maxDepth: maximum desired search depth - once reached, your code should somehow return
			  a minimax utility value for this location.

		alpha. beta: alpha and beta values passed from the parent node to constrain search at this
			     level.

   Return values:
		Your search code will directly update data passed-in as arguments:
		
		- Mini-Max value	: Notice this function returns a double precision number. This is
					  the minimax value at this level of the tree. It will be used 
					  as the recursion backtracks filling-in the mini-max values back
					  from the leaves to the root of the search tree. 

		- path[1][2]		: Your MiniMax function will return the location for the agent's 
					  next location (i.e. the optimal move for the agent). 
		- minmax_cost[size_X][size_Y] 	:  Your search code will update this array to contain the
						   minimax value for locations that were expanded during
						   the search. This must be done *only* for the mouse.

						   Values in this array will be in the range returned by
						   your utility function.

		* Your code MUST NOT modify the locations or numbers of cats and/or cheeses, the graph,
	     	  or the location of the mouse - if you try, the driver code will know it *
			
		That's that, now, implement your solution!
 */

 /********************************************************************************************************
 * 
 * TO DO:	Implement code to perform a MiniMax search. This will involve a limited-depth BFS-like
 *              expansion. Once nodes below return values, your function will propagate minimax utilities
 *		as per the minimax algorithm.
 *	
 *		Note that if alpha-beta pruning is specified, you must keep track of alphas and betas
 *		along the path.
 *
 *		You can use helper functions if it seems reasonable. Add them to the MiniMax_search.h
 *		file and explain in your code why they are needed and how they are used.
 *
 *		Recursion should appear somewhere.
 *
 *		MiniMax cost: If the agentId=0 (Mouse), then once you have a MiniMax value for a location
 *		in the maze, you must update minmax_cost[][] for that location.
 *
 *		How you design your solution is up to you. But:
 *
 *		- Document your implementation by adding concise and clear comments in this file
 *		- Document your design (how you implemented the solution, and why) in the report
 *
 ********************************************************************************************************/

 // Stub so that the code compiles/runs - This will be removed and replaced by your code!
 static int called;
 if (!called)
	 shortest_paths(gr);
 called = 1;

 int cat_index;
 int mouse_index = mouse_loc[0][0] + mouse_loc[0][1]*size_X;
 int new_cat_loc[10][2];
 int new_mouse_loc[1][2];

 for(int i; i<cats; i++){
	 new_cat_loc[i][0] = cat_loc[i][0];
	 new_cat_loc[i][1] = cat_loc[i][1];
 }
 
 new_mouse_loc[0][0] = mouse_loc[0][0];
 new_mouse_loc[0][1] = mouse_loc[0][1];

double up,right,down,left;
int up_called = 0, right_called = 0, down_called = 0, left_called = 0;
double max_value, min_value;
int max_index,min_index;

 if (mode == 0) {
	 if (depth == maxDepth || checkForTerminal(mouse_loc, cat_loc, cheese_loc, cats, cheeses)) {
		 double u = utility(cat_loc, cheese_loc, mouse_loc, cats, cheeses, depth, gr);
		 //if (checkForTerminal(mouse_loc, cat_loc, cheese_loc, cats, cheeses)) printf("this is the terminal value %f\n", u);
		 return u;
	 }

	 for (int i = 0; i<4; i++){
		 if (agentId == 0) {
			if (gr[mouse_index][i]) {
				if (i==0) {
					//up
					new_mouse_loc[0][1]--;
					up = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = up;
					new_mouse_loc[0][1]++;
				} else if (i == 1) {
					//right
					new_mouse_loc[0][0]++;
					right = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = right;
					new_mouse_loc[0][0]--;
				} else if (i == 2) {
					//down
					new_mouse_loc[0][1]++;
					down = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = down;
					new_mouse_loc[0][1]--;
				} else if (i == 3) {
					//left
					new_mouse_loc[0][0]--;
					left = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = left;
					new_mouse_loc[0][0]++;
				}
			} else {
				if (i==0) {
					up = -1000000000;
				} else if (i==1) {
					right = -1000000000;
				} else if (i==2) {
					down = -1000000000;
				} else if (i==3) {
					left = -1000000000;
				}
			}
		 } else {
			 cat_index = cat_loc[agentId - 1][0] + cat_loc[agentId - 1][1]*size_X;
			 int new_agentId = agentId + 1;

			 if (new_agentId == cats + 1) {
				 new_agentId = 0;
			 }
			 if (gr[cat_index][i]) {
				 if (i == 0) {
					//up
					new_cat_loc[agentId - 1][1]--;
					up = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					new_cat_loc[agentId - 1][1]++;
				 } else if (i == 1) {
					//right
					new_cat_loc[agentId - 1][0]++;
					right = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					new_cat_loc[agentId - 1][0]--;
				 } else if (i == 2) {
					//down
					new_cat_loc[agentId - 1][1]++;
					down = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					new_cat_loc[agentId - 1][1]--;
				 } else if (i == 3) {
					//left 
					new_cat_loc[agentId - 1][0]--;
					left = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					new_cat_loc[agentId - 1][0]++;
				 }
			 }else {
				if (i==0) {
					up = 1000000000;
				} else if (i==1) {
					right = 1000000000;
				} else if (i==2) {
					down = 1000000000;
				} else if (i==3) {
					left = 1000000000;
				}
			}
		 }
	 }

	 if (agentId == 0) {
		set_index_val_max(up, right, down, left);
		if(depth == 0) {
			// printf("\nwe are a mouse %f %f %f %f\n",up,right,down,left);
			// for (int i = 0; i < cheeses; i++)
			// {
			// 	printf("cheese %d is at (%d,%d)\n", i, cheese_loc[i][0],cheese_loc[i][1]);
			// }
			// 	printf("target_cheese is (%d,%d)\n", target_cheese % size_X, target_cheese/size_X);

		// {
		// 		int target_cheese = 0;
		// 	int cur_cat_loc = 0;
		// 	int cur_cheese_loc = 0;
		// 	int cheese_mouse_value[cheeses];
		// 	int mouse_location = mouse_loc[0][0] + mouse_loc[0][1]*size_X;

		// 	for(int i = 0; i < cheeses; i++){
		// 		for(int j = 0; j < cats; j++) {
		// 			cur_cheese_loc = cheese_loc[i][0] + cheese_loc[i][1]*size_X;
		// 			cur_cat_loc = cat_loc[j][0] + cat_loc[j][1]*size_X;
		// 			cheese_mouse_value[i] += shortest_matrix[cur_cheese_loc][cur_cat_loc];
		// 		}
		// 		cheese_mouse_value[i] += (graph_size+1)/(shortest_matrix[cur_cheese_loc][mouse_location]+1);
		// 	}

		// 	int max = -1;
		// 	for(int i = 0; i < cheeses; i++){
		// 		cur_cheese_loc = cheese_loc[i][0] + cheese_loc[i][1]*size_X;
		// 		if(cheese_mouse_value[i] > max){
		// 			max = cheese_mouse_value[i];
		// 			target_cheese = cur_cheese_loc;
		// 		}
		// 	}
		// 	printf("target cheese is %d\n",target_cheese);
		// }
			// printf("max is index %f and value %f\n",index_val[0],index_val[1]);
			// printf ("we are at location (%d,%d)\n\n",mouse_loc[0][0],mouse_loc[0][1]);

			
		}
		if (index_val[0] == 0) {
			path[0][0] = mouse_loc[0][0];
			path[0][1] = mouse_loc[0][1] - 1;
		} else if (index_val[0] == 1) {
			path[0][0] = mouse_loc[0][0] + 1;
			path[0][1] = mouse_loc[0][1];
		} else if (index_val[0] == 2) {
			path[0][0] = mouse_loc[0][0];
			path[0][1] = mouse_loc[0][1] + 1;
		} else if (index_val[0] == 3) {
			path[0][0] = mouse_loc[0][0] - 1;
			path[0][1] = mouse_loc[0][1];
		}
	 } else {
		set_index_val_min(up, right, down, left);
		if (index_val[0] == 0) {
			path[0][0] = cat_loc[agentId - 1][0];
			path[0][1] = cat_loc[agentId - 1][1] - 1;
		} else if (index_val[0] == 1) {
			path[0][0] = cat_loc[agentId - 1][0] + 1;
			path[0][1] = cat_loc[agentId - 1][1];
		} else if (index_val[0] == 2) {
			path[0][0] = cat_loc[agentId - 1][0];
			path[0][1] = cat_loc[agentId - 1][1] + 1;
		} else if (index_val[0] == 3) {
			path[0][0] = cat_loc[agentId - 1][0] - 1;
			path[0][1] = cat_loc[agentId - 1][1];
		}
	 }

	return index_val[1];

 } else if (mode == 1) {
 	 if (depth == maxDepth || checkForTerminal(mouse_loc, cat_loc, cheese_loc, cats, cheeses)) {
		 double u = utility(cat_loc, cheese_loc, mouse_loc, cats, cheeses, depth, gr);
		 return u;
	 }

	 if (agentId == 0){
		 double max_value = -1000000000;
		 
		 for (int i = 0; i < 4; i++){
			 if (gr[mouse_index][i]){
				 				 
				if (i==0) {
					new_mouse_loc[0][1]--;
					up = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = up;
					
					if (up > max_value){
						max_value = up;
						max_index = i;
					}
					new_mouse_loc[0][1]++;
					alpha = max_value > alpha ? max_value : alpha;
					if (max_value >= beta) break;
					
				} else if (i==1){

					new_mouse_loc[0][0]++;
					right = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = right;

					if (right > max_value){
						max_value = right;
						max_index = i;
					}

					new_mouse_loc[0][0]--;
					alpha = max_value > alpha ? max_value : alpha;
					if (max_value >= beta) break;

				} else if (i==2){
					new_mouse_loc[0][1]++;
					down = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = down;

					if (down > max_value){
						max_value = down;
						max_index = i;
					}

					new_mouse_loc[0][1]--;

					alpha = max_value > alpha ? max_value : alpha;
					if (max_value >= beta) break;

				} else if (i==3){
					
					new_mouse_loc[0][0]--;
					left = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = left;

					if (left > max_value){
						max_value = left;
						max_index = i;
					}
					
					new_mouse_loc[0][0]++;
					alpha = max_value > alpha ? max_value : alpha;
					if (max_value >= beta) break;
				}
			 }	 
		 }
		 if (max_index == 0){
			path[0][0] = mouse_loc[0][0];
			path[0][1] = mouse_loc[0][1] - 1;
		 } else if (max_index == 1) {
			path[0][0] = mouse_loc[0][0] + 1;
			path[0][1] = mouse_loc[0][1];
		 } else if (max_index == 2) {
			 path[0][0] = mouse_loc[0][0];
			 path[0][1] = mouse_loc[0][1] + 1;
		 } else if (max_index == 3) {
			path[0][0] = mouse_loc[0][0] - 1;
			path[0][1] = mouse_loc[0][1];
		 }
		 
		 return max_value;
	 }
	 else{
		 double min_value = 1000000000;
		 cat_index = cat_loc[agentId - 1][0] + cat_loc[agentId - 1][1] * size_X;
		 int new_agentId = agentId + 1;
		 if (new_agentId == cats + 1){
			 new_agentId = 0;
		 }

		 for (int i = 0; i < 4; i++){	
			if (gr[cat_index][i]){
				if (i==0) {
					new_cat_loc[agentId - 1][1]--;
					up = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					
					if (up < min_value){
						min_value = up;
						min_index = i;
					}

					new_cat_loc[agentId - 1][1]++;

					beta = min_value < beta ? min_value : beta;
					if (min_value <= alpha) break;
				} else if (i==1) {
					new_cat_loc[agentId - 1][0]++;
					right = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					
					if (right < min_value){
						min_value = right;
						min_index = i;
					}

					new_cat_loc[agentId - 1][0]--;

					beta = min_value < beta ? min_value : beta;
					if (min_value <= alpha) break;
				} else if (i==2) {
					new_cat_loc[agentId - 1][1]++;
					down = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					
					if (down < min_value){
						min_value = down;
						min_index = i;
					}

					new_cat_loc[agentId - 1][1]--;

					beta = min_value < beta ? min_value : beta;
					if (min_value <= alpha) break;
				} else if (i==3) {
					new_cat_loc[agentId - 1][0]--;
					left = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					
					if (left < min_value){
						min_value = left;
						min_index = i;
					}

					new_cat_loc[agentId - 1][0]++;

					beta = min_value < beta ? min_value : beta;
					if (min_value <= alpha) break;
				}
			}
		 }
		 if (max_index == 0){
			path[0][0] = cat_loc[agentId - 1][0];
			path[0][1] = cat_loc[agentId - 1][1] - 1;
		 } else if (max_index == 1) {
			path[0][0] = cat_loc[agentId - 1][0] + 1;
			path[0][1] = cat_loc[agentId - 1][1];
		 } else if (max_index == 2) {
			path[0][0] = cat_loc[agentId - 1][0];
			path[0][1] = cat_loc[agentId - 1][1] + 1;
		 } else if (max_index == 3) {
			path[0][0] = cat_loc[agentId - 1][0] - 1;
			path[0][1] = cat_loc[agentId - 1][1];
		 }
		 
		 return min_value;
	 }

/*
	 for (int i = 0; i<4; i++){
		 if (agentId == 0) {
			if (gr[mouse_index][i] && alpha < beta) {
				if (i==0) {
					//up
					up_called = 1;
					new_mouse_loc[0][1]--;
					up = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = up;
					new_mouse_loc[0][1]++;

					if (up > alpha) alpha = up;
					if (alpha >= beta){
						right = -1000000000;
						down = -1000000000;
						left =  -1000000000;
					}
				} else if (i == 1) {
					//right
					right_called = 1;
					new_mouse_loc[0][0]++;
					right = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = right;
					new_mouse_loc[0][0]--;

					if (right > alpha) alpha = right;

					if (alpha >= beta){
						down = -1000000000;
						left = -1000000000;
						if(up_called) right = -1000000000;
					}
				} else if (i == 2) {
					//down
					down_called = 1;
					new_mouse_loc[0][1]++;
					down = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = down;
					new_mouse_loc[0][1]--;

					if (down > alpha) alpha = down;
					if (alpha >= beta) {
						left = -1000000000;
						if (up_called || right_called) down = -1000000000;
					}
				} else if (i == 3) {
					//left
					left_called = 1;
					new_mouse_loc[0][0]--;
					left = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, agentId + 1, depth + 1, maxDepth, alpha, beta);
					minmax_cost[new_mouse_loc[0][0]][new_mouse_loc[0][1]] = left;
					new_mouse_loc[0][0]++;

					if (left > alpha) alpha = left;
					//if (alpha >= beta) left = -1000000000;
				}
			} else {
				if (i==0) {
					up = -1000000000;
				} else if (i==1) {
					right = -1000000000;
				} else if (i==2) {
					down = -1000000000;
				} else if (i==3) {
					left = -1000000000;
				}
			}
		 } else {
			 cat_index = cat_loc[agentId - 1][0] + cat_loc[agentId - 1][1]*size_X;
			 int new_agentId = agentId + 1;

			 if (new_agentId == cats + 1) {
				 new_agentId = 0;
			 }
			 if (gr[cat_index][i] && beta > alpha) {
				 if (i == 0) {
					//up
					new_cat_loc[agentId - 1][1]--;
					up = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					new_cat_loc[agentId - 1][1]++;

					if (up < beta) beta = up;
					if (alpha >= beta){
						right = 1000000000;
						down = 1000000000;
						left = 1000000000;
					}
				 } else if (i == 1) {
					//right
					new_cat_loc[agentId - 1][0]++;
					right = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					new_cat_loc[agentId - 1][0]--;

					if (right < beta) beta = right;
					if (alpha >= beta){
						down = 1000000000;
						left = 1000000000;
					}
				 } else if (i == 2) {
					//down
					new_cat_loc[agentId - 1][1]++;
					down = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					new_cat_loc[agentId - 1][1]--;

					if (down < beta) beta = down;
					if (alpha >= beta) down = 1000000000;
				 } else if (i == 3) {
					//left 
					new_cat_loc[agentId - 1][0]--;
					left = MiniMax(gr, path, minmax_cost, new_cat_loc, cats, cheese_loc, cheeses, new_mouse_loc, mode, utility, new_agentId, depth + 1, maxDepth, alpha, beta);
					new_cat_loc[agentId - 1][0]++;

					if (left < beta) beta = left;
					if (alpha >= beta) left = 1000000000;
				 }
			 }else {
				if (i==0) {
					up = 1000000000;
				} else if (i==1) {
					right = 1000000000;
				} else if (i==2) {
					down = 1000000000;
				} else if (i==3) {
					left = 1000000000;
				}
			}
		 }
	 }

	 if (agentId == 0) {
		set_index_val_max(up, right, down, left);
		if (index_val[0] == 0) {
			path[0][0] = mouse_loc[0][0];
			path[0][1] = mouse_loc[0][1] - 1;
		} else if (index_val[0] == 1) {
			path[0][0] = mouse_loc[0][0] + 1;
			path[0][1] = mouse_loc[0][1];
		} else if (index_val[0] == 2) {
			path[0][0] = mouse_loc[0][0];
			path[0][1] = mouse_loc[0][1] + 1;
		} else if (index_val[0] == 3) {
			path[0][0] = mouse_loc[0][0] - 1;
			path[0][1] = mouse_loc[0][1];
		}
	 } else {
		set_index_val_min(up, right, down, left);
		if (index_val[0] == 0) {
			path[0][0] = cat_loc[agentId - 1][0];
			path[0][1] = cat_loc[agentId - 1][1] - 1;
		} else if (index_val[0] == 1) {
			path[0][0] = cat_loc[agentId - 1][0] + 1;
			path[0][1] = cat_loc[agentId - 1][1];
		} else if (index_val[0] == 2) {
			path[0][0] = cat_loc[agentId - 1][0];
			path[0][1] = cat_loc[agentId - 1][1] + 1;
		} else if (index_val[0] == 3) {
			path[0][0] = cat_loc[agentId - 1][0] - 1;
			path[0][1] = cat_loc[agentId - 1][1];
		}
	 }

	if(agentId == 0) {
		minmax_cost[path[0][0]][path[0][1]] = index_val[1];
	}

	return index_val[1];
 }
*/
 }

}

void set_index_val_max(double first, double second, double third, double fourth) {
	if(first >= second && first >= third && first >= fourth){
		index_val[0] = 0;
		index_val[1] = first;
	}
	if(second >= first && second >= third && second >= fourth){
		index_val[0] = 1;
		index_val[1] = second;
	}
	if(third >= first && third >= second && third >= fourth){
		index_val[0] = 2;
		index_val[1] = third;
	}
	if(fourth >= first && fourth >= second && fourth >= third){
		index_val[0] = 3;
		index_val[1] = fourth;
	}
}

void set_index_val_min(double first, double second, double third, double fourth) {
	if(first <= second && first <= third && first <= fourth){
		index_val[0] = 0;
		index_val[1] = first;
	}
	if(second <= first && second <= third && second <= fourth){
		index_val[0] = 1;
		index_val[1] = second;
	}
	if(third <= first && third <= second && third <= fourth){
		index_val[0] = 2;
		index_val[1] = third;
	}
	if(fourth <= first && fourth <= second && fourth <= third){
		index_val[0] = 3;
		index_val[1] = fourth;
	}
}

double utility(int cat_loc[10][2], int cheese_loc[10][2], int mouse_loc[1][2], int cats, int cheeses, int depth, double gr[graph_size][4])
{
 /*
	This function computes and returns the utility value for a given game configuration.
	As discussed in lecture, this should return a positive value for configurations that are 'good'
	for the mouse, and a negative value for locations that are 'bad' for the mouse.

	How to define 'good' and 'bad' is up to you. Note that you can write a utility function
	that favours your mouse or favours the cats, but that would be a bad idea... (why?)

	Input arguments:

		cat_loc - Cat locations
		cheese_loc - Cheese locations
		mouse_loc - Mouse location
		cats - # of cats
		cheeses - # of cheeses
		depth - current search depth
		gr - The graph's adjacency list for the maze

		These arguments are as described in A1. Do have a look at your solution!
 */
 	target_cheese = 0;
	int cur_cat_loc = 0;
	int cur_cheese_loc = 0;
	int cheese_mouse_value[cheeses];
	int mouse_location = mouse_loc[0][0] + mouse_loc[0][1]*size_X;

	for(int i = 0; i < cheeses; i++){
		cur_cheese_loc = cheese_loc[i][0] + cheese_loc[i][1]*size_X;
		// for(int j = 0; j < cats; j++) {
		// 	cur_cheese_loc = cheese_loc[i][0] + cheese_loc[i][1]*size_X;
		// 	cur_cat_loc = cat_loc[j][0] + cat_loc[j][1]*size_X;
		// 	cheese_mouse_value[i] += shortest_matrix[cur_cheese_loc][cur_cat_loc];
		// }
		cheese_mouse_value[i] = (shortest_matrix[cur_cheese_loc][mouse_location]); // USED TRO BE +=
	}

	int min = 1000000000;
	for(int i = 0; i < cheeses; i++){
		cur_cheese_loc = cheese_loc[i][0] + cheese_loc[i][1]*size_X;
		if(cheese_mouse_value[i] < min){
			min = cheese_mouse_value[i];
			target_cheese = cur_cheese_loc;
		}
	}

	int mouse_to_cheese = shortest_matrix[mouse_location][target_cheese];

	int distance_from_cats = 0;
	for(int i = 0; i < cats; i++){
		cur_cat_loc = cat_loc[i][0] + cat_loc[i][1]*size_X;
		distance_from_cats += shortest_matrix[mouse_location][cur_cat_loc];
	}

	int min_distance_from_cats = INF;
	for(int i = 0; i < cats; i++){
		cur_cat_loc = cat_loc[i][0] + cat_loc[i][1]*size_X;
		if(shortest_matrix[mouse_location][cur_cat_loc] < min_distance_from_cats)
			min_distance_from_cats = shortest_matrix[mouse_location][cur_cat_loc];
	}

	// if(fabs((1/(mouse_to_cheese + 0.001) - 1/((min_distance_from_cats+0.01)*(min_distance_from_cats+0.01)))) < 0.1){
	// 	printf("00\n");
	// }
	// printf("%f\n",fabs(((graph_size*10)/(mouse_to_cheese + 0.001) - (graph_size*10)/((min_distance_from_cats+0.01)*(min_distance_from_cats+0.01)))));

	// double value = ((graph_size*10)/(mouse_to_cheese + 0.001) - (graph_size*10)/((min_distance_from_cats+0.01)*(min_distance_from_cats+0.01)));
	// if(value < -5000) value = -5000;
	// if(value > 5000) value = 5000;
	// return value;
	//return ((graph_size*10)/(mouse_to_cheese + 0.01) - (graph_size*10)/((min_distance_from_cats+0.01)));

	// double value = (1.5/(mouse_to_cheese + 0.001) - 1/((min_distance_from_cats+0.01)*(min_distance_from_cats+0.01)));
	// if(value < -1499) value = -1499;
	// if(value > 1499) value = 1499;
	// return value;
	
	return (1.5/(mouse_to_cheese + 0.001) - 1/((min_distance_from_cats+0.01)*(min_distance_from_cats+0.01)));
}

int checkForTerminal(int mouse_loc[1][2],int cat_loc[10][2],int cheese_loc[10][2],int cats,int cheeses)
{
 /* 
   This function determines whether a given configuration constitutes a terminal node.
   Terminal nodes are those for which:
     - A cat eats the mouse
     or
     - The mouse eats a cheese
   
   If the node is a terminal, the function returns 1, else it returns 0
 */

 // Check for cats having lunch
 for (int i=0; i<cats; i++)
  if (mouse_loc[0][0]==cat_loc[i][0]&&mouse_loc[0][1]==cat_loc[i][1]) return(1);

 // Check for mouse having lunch
 for (int i=0; i<cheeses; i++)
  if (mouse_loc[0][0]==cheese_loc[i][0]&&mouse_loc[0][1]==cheese_loc[i][1]) return(1);

 return(0);

}

//-------CODE FOR SHORTEST PATHS----REFRENCE FROM https://www.programiz.com/dsa/floyd-warshall-algorithm
void shortest_paths(double gr[graph_size][4]){
	int graph[graph_size][graph_size];

    for(int i = 0; i < graph_size; i++){
        for(int j = 0; j < graph_size; j++){
            graph[i][j] = INF;
        }
    }

	for(int i = 0; i < graph_size; i++){
        if(gr[i][1]) graph[i][i+1] = 1;
		if(gr[i][3]) graph[i][i-1] = 1;
		if(gr[i][0]) graph[i][i-size_X] = 1;
		if(gr[i][2]) graph[i][i+size_X] = 1;
		graph[i][i] = 0;
    }

  int i, j, k;

  for (i = 0; i < graph_size; i++)
    for (j = 0; j < graph_size; j++)
      shortest_matrix[i][j] = graph[i][j];

  for (k = 0; k < graph_size; k++) {
    for (i = 0; i < graph_size; i++) {
      for (j = 0; j < graph_size; j++) {
        if (shortest_matrix[i][k] + shortest_matrix[k][j] < shortest_matrix[i][j])
          shortest_matrix[i][j] = shortest_matrix[i][k] + shortest_matrix[k][j];
      }
    }
  }
}
//-----------DONE SHORTEST PATHS CODE---------

