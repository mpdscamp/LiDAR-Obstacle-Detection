/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		int limitX = 0;

		Node* newNode(new Node(point, id));
		Node* currentNode = root;

		while (currentNode) {
			if (point[limitX] < currentNode->point[limitX]) {
				if (currentNode->left == nullptr) {
					currentNode->left = newNode;
					break;
				}
				currentNode = currentNode->left;
			}
			else {
				if (currentNode->right == nullptr) {
					currentNode->right = newNode;
					break;
				}
				currentNode = currentNode->right;
			}

			limitX = 1 - limitX;
		}

		if (root == nullptr) {
			root = newNode;
		}
	}

	void searchHelper(Node* node, std::vector<float> target, float distanceTol, int depth, std::vector<int>& ids) {
		if (node == nullptr) {
			return;
		}
		
		if (node->point[0] >= target[0] - distanceTol && node->point[0] <= target[0] + distanceTol &&
			node->point[1] >= target[1] - distanceTol && node->point[1] <= target[1] + distanceTol) {
			
			float distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2));

			if (distance <= distanceTol) {
				ids.push_back(node->id);
			}
		}

		if (target[depth % 2] - distanceTol < node->point[depth % 2]) {
			searchHelper(node->left, target, distanceTol, depth + 1, ids);
		}
		if (target[depth % 2] + distanceTol > node->point[depth % 2]) {
			searchHelper(node->right, target, distanceTol, depth + 1, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		
		return ids;
	}
};