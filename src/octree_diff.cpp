#include <cmath>
#include <set>
#include <sstream>

#include <octomap/octomap.h>

#include "octree_diff.h"


std::pair<octomap::OcTree, octomap::OcTree> calcOctreeDiff(
        const octomap::OcTree& tree1,
        const octomap::OcTree& tree2,
        const double updateEps
) {
    double resolution = tree1.getResolution();
    // pure diff between tree1 and tree2 (can have negative values)
    octomap::OcTree diffTree(resolution);
    // values of nodes in tree2 that are different from tree1 (can't have negative values)
    octomap::OcTree updateTree(resolution);

    std::set<std::string> checkedNodes;

    for (octomap::OcTree::leaf_iterator it = tree2.begin_leafs(), end = tree2.end_leafs(); it != end; ++it) {
        // current node's occupancy odds
        double logOdds = it->getLogOdds();
        double occupancyUpdate = logOdds;
        
        // current node's coordinates
        octomap::point3d nodeCoords = it.getCoordinate();
        bool nodeOccupied = it->getValue();

        // current node in tree1
        octomap::OcTreeNode* nodeInTree1 = tree1.search(nodeCoords);
        // current node from tree2 found in tree1
        if (nodeInTree1 != nullptr) {
            occupancyUpdate -= nodeInTree1->getLogOdds();
        }
        // record update if it's above threshold
        if (std::fabs(occupancyUpdate) >= updateEps) {
            octomap::OcTreeNode* updateNode = updateTree.updateNode(nodeCoords, nodeOccupied);
            updateNode->setLogOdds(logOdds);
            
            octomap::OcTreeNode* diffNode = diffTree.updateNode(nodeCoords, nodeOccupied);
            diffNode->setLogOdds(occupancyUpdate);
        }

        std::ostringstream oss;
        oss << nodeCoords;
        checkedNodes.insert(oss.str());
    }

    for (octomap::OcTree::leaf_iterator it = tree1.begin_leafs(), end = tree1.end_leafs(); it != end; ++it) {
        octomap::point3d nodeCoords = it.getCoordinate();
        std::ostringstream oss;
        oss << nodeCoords;
        std::string nodeStr = oss.str();

        if (checkedNodes.find(nodeStr) == checkedNodes.end()) {
            // current node was not checked in the 1st loop, means it's not present in tree2
            double occupancyUpdate = -(it->getLogOdds());
            if (std::fabs(occupancyUpdate) >= updateEps) {
                octomap::OcTreeNode* diffNode = diffTree.updateNode(nodeCoords, false);
                diffNode->setLogOdds(occupancyUpdate);
            }
        }
    }

    return std::make_pair(updateTree, diffTree);
}
