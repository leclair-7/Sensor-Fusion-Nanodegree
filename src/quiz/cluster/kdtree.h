/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KHTREE_H
#define KHTREE_H

#include "../../render/render.h"
#include <cmath>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	float x()
	{
		if (point.empty() )
		{
			std::cout << "empty node, returning\n";
			return -1;
		}
		return point[0];
	}
	float y()
	{
		if (point.empty() || point.size() < 2 )
		{
			std::cout << "empty node, returning\n";
			return -1;
		}
		return point[1];
	}
	float z()
	{
		if (point.empty() || point.size() < 3 )
		{
			std::cout << "empty node, returning\n";
			return -1;
		}
		return point[2];
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		cout << "insert call\n";
		if ( !root)
		{
			std::cout << "created root node " << id << "\n";
			Node * leaf = new Node(point,id);
			root=leaf;
		}
		else
		{
			Node * leaf = new Node(point,id);
			insertHelper(leaf,root,0);
		}
	}
	void insertHelper(Node * apNode, Node * treeNodeToCompare, int depth)
	{

		int splitOn(depth%3);
		if ( splitOn == 0)
		{
			//compare x
			if ( apNode->x() > treeNodeToCompare->x()  )
			{
				if ( treeNodeToCompare->right )
				{
					insertHelper(apNode,treeNodeToCompare->right,depth+1);
				}
				else
				{

					treeNodeToCompare->right = apNode;
				}
			}
			else
			{
				if ( treeNodeToCompare->left )
				{
					insertHelper(apNode,treeNodeToCompare->left,depth+1);
				}
				else
				{
					treeNodeToCompare->left = apNode;
				}
			}
		}
		else if ( splitOn == 1)
		{
			//compare y
			if ( apNode->y() > treeNodeToCompare->y()  )
			{
				if ( treeNodeToCompare->right )
				{
					insertHelper(apNode,treeNodeToCompare->right,depth+1);
				}
				else
				{
					treeNodeToCompare->right = apNode;
				}
			}
			else
			{
				if ( treeNodeToCompare->left )
				{
					insertHelper(apNode,treeNodeToCompare->left,depth+1);
				}
				else
				{
					treeNodeToCompare->left = apNode;
				}
			}
		}
		else
		{
			//compare y
			if ( apNode->z() > treeNodeToCompare->z()  )
			{
				if ( treeNodeToCompare->right )
				{
					insertHelper(apNode,treeNodeToCompare->right,depth+1);
				}
				else
				{
					treeNodeToCompare->right = apNode;
				}
			}
			else
			{
				if ( treeNodeToCompare->left )
				{
					insertHelper(apNode,treeNodeToCompare->left,depth+1);
				}
				else
				{
					treeNodeToCompare->left = apNode;
				}
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,root,distanceTol,0,ids);
		return ids;
	}
	void searchHelper(std::vector<float>target, Node* pNodeTest, float distanceTol, int depth, std::vector<int>&ids)
	{
		if (pNodeTest)
		{
			if( pNodeTest->x() >= (target[0]-distanceTol)&&(pNodeTest->x()<=(target[0]+distanceTol))
			 && pNodeTest->y() >= (target[1]-distanceTol)&&(pNodeTest->y()<=(target[1]+distanceTol))
			 && pNodeTest->z() >= (target[2]-distanceTol)&&(pNodeTest->z()<=(target[2]+distanceTol)) )
			 {
				float dist = sqrt(pow(pNodeTest->x()-target[0],2) + pow(pNodeTest->y()-target[1],2) + pow(pNodeTest->z()-target[2],2));
				if ( dist < distanceTol)
				{
					ids.push_back(pNodeTest->id);
				}
			 }

			if ( (target[depth%3] - distanceTol)<pNodeTest->point[depth%3])
			{
				searchHelper(target,pNodeTest->left,distanceTol,depth+1,ids);
			}
			if ( (target[depth%3] + distanceTol)>pNodeTest->point[depth%3])
			{
				searchHelper(target,pNodeTest->right,distanceTol,depth+1,ids);
			}
		}
	}
	


};

#endif // KHTREE_H


