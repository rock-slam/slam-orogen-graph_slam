#ifndef __GRAPH_SLAM_POSE_GRAPH_HPP__
#define __GRAPH_SLAM_POSE_GRAPH_HPP__

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/Featurecloud.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/operators/DistanceGridToPointcloud.hpp>

#include <aislib/graph_optimizer/graph_optimizer3d_hchol.h>
#include <envire/ransac.hpp>
#include <envire/icpConfigurationTypes.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <stereo/sparse_stereo.hpp>

namespace graph_slam
{

Transformation3 eigen2Hogman( const Eigen::Affine3d& eigen_transform )
{
    Eigen::Quaternionf eigen_quat(eigen_transform.rotation().cast<float>());
    Eigen::Matrix4d eigen_mat( eigen_transform.matrix() );
    Vector3 translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
    Quaternion rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(),
	    eigen_quat.w());
    Transformation3 result(translation, rotation);

    return result;
}

Matrix6 eigen2Hogman( const Eigen::Matrix<double,6,6>& eigen_matrix )
{
    Matrix6 result;
    // TODO might be more efficient to perform a memcopy here
    for( int m=0; m<6; m++ )
	for( int n=0; n<6; n++ )
	    result[m][n] = eigen_matrix(m,n);

    return result;
}

Eigen::Matrix<double,6,6> hogman2Eigen( const Matrix6& hogman )
{
    Eigen::Matrix<double,6,6> result;

    for( int m=0; m<6; m++ )
	for( int n=0; n<6; n++ )
	    result(m,n) = hogman[m][n];

    return result;
}

Eigen::Affine3d hogman2Eigen( const Transformation3& hogman_transform )
{
    Eigen::Quaterniond rotation( 
	    hogman_transform.rotation().w(),
	    hogman_transform.rotation().x(),
	    hogman_transform.rotation().y(),
	    hogman_transform.rotation().z() );

    Eigen::Translation3d translation(
	    hogman_transform.translation().x(),
	    hogman_transform.translation().y(),
	    hogman_transform.translation().z() );

    return translation * rotation;
}

/** 
 * transform a 6x6 covariance matrix in [r t] order to a hogman 6 inverse
 * covariance matrix in [t r] format.
 */
Matrix6 envireCov2HogmanInf( const Eigen::Matrix<double,6,6>& eigen_matrix )
{
    Eigen::Matrix<double,6,6> t;
    t << eigen_matrix.bottomRightCorner<3,3>(), eigen_matrix.bottomLeftCorner<3,3>(),
      eigen_matrix.topRightCorner<3,3>(), eigen_matrix.topLeftCorner<3,3>();

    return eigen2Hogman( Eigen::Matrix<double,6,6>(t.inverse()) );
}

/** 
 * Transform a 6x6 covariance matrix in [t r] order from hogman Covariance
 * format to the envire [r t] order.
 */
Eigen::Matrix<double,6,6> hogmanCov2EnvireCov( const Matrix6& hogman_matrix )
{
    Eigen::Matrix<double,6,6> t = hogman2Eigen( hogman_matrix );
    Eigen::Matrix<double,6,6> t1;
    t1 << t.bottomRightCorner<3,3>(), t.bottomLeftCorner<3,3>(),
      t.topRightCorner<3,3>(), t.topLeftCorner<3,3>();
    return t1;
}

class PoseGraph
{
public:
    class SensorMaps;

protected:
    /** maximum distance between nodes, where we check for potential
     * correspondence 
     */
    double max_node_radius;

    /** minimum number of sparse correspondences required to consider a match
     * successfull.
     */
    size_t min_sparse_correspondences;

protected:
    envire::Environment *env;
    AISNavigation::GraphOptimizer3D *optimizer;
    std::map<long, SensorMaps*> nodeMap;

    // body frame of the robot
    envire::FrameNode::Ptr bodyFrame;

    // chain for processing distance images
    envire::FrameNode::Ptr distFrame;
    envire::DistanceGrid::Ptr distGrid;
    envire::DistanceGridToPointcloud::Ptr distOp;

    // chain for feature clouds
    envire::FrameNode::Ptr featureFrame;
    envire::Featurecloud::Ptr featurecloud;

    // current node
    envire::FrameNode::Ptr prevBodyFrame;
    envire::FrameNode::Ptr currentBodyFrame;

public:
    PoseGraph( envire::Environment* env, int num_levels = 3, int node_distance = 2 ) 
	: max_node_radius( 25.0 ), 
	min_sparse_correspondences( 7 ),
	env( env ), optimizer( new AISNavigation::HCholOptimizer3D( num_levels, node_distance ) ) 
    {
	// set-up body frame
	bodyFrame = new envire::FrameNode();
	env->addChild( env->getRootNode(), bodyFrame.get() );

	// set-up chain for distance images
	distFrame = new envire::FrameNode();
	env->addChild( bodyFrame.get(), distFrame.get() );

	distOp = new envire::DistanceGridToPointcloud();
	distOp->setMaxDistance( 5.0 );
	env->attachItem( distOp.get() );

	// set-up the chain for the feature clouds
	featureFrame = new envire::FrameNode();
	env->addChild( bodyFrame.get(), featureFrame.get() );
	featurecloud = new envire::Featurecloud();
	env->setFrameNode( featurecloud.get(), featureFrame.get() );
    }

    ~PoseGraph()
    {
	// do some cleanup
	delete optimizer;

	// delete the nodeMap objects
	for( std::map<long, SensorMaps*>::iterator it = nodeMap.begin(); it != nodeMap.end(); it++ )
	    delete it->second;
    }

    /** will prepare a new node based on an initial transformation
     */
    void initNode( const envire::TransformWithUncertainty &body2bodyPrev )
    {
	// apply the relative transform to the current bodyFrame position
	bodyFrame->setTransform( bodyFrame->getTransformWithUncertainty() *
		body2bodyPrev );

	// copy the bodyFrame node, and use it to store any additional map information
	// that is added through the other add functions
	currentBodyFrame = new envire::FrameNode(
		bodyFrame->getTransformWithUncertainty() );

	env->addChild( bodyFrame->getParent(), currentBodyFrame.get() );

	// create a new hogman vertex for this node
	AISNavigation::PoseGraph3D::Vertex *new_vertex = optimizer->addVertex( 
		currentBodyFrame->getUniqueId(),
		Transformation3(),
		Matrix6::eye(1.0) );

	// in case there is a previous node create a vertex based on odometry
	// information
	if( prevBodyFrame )
	{
	    optimizer->addEdge( 
		    optimizer->vertex( prevBodyFrame->getUniqueId() ),
		    new_vertex,
		    eigen2Hogman( body2bodyPrev.getTransform() ),
		    envireCov2HogmanInf( body2bodyPrev.getCovariance() )
		    );
	}
    }

    /** adds a sensor reading for a distance image to an initialized node
     */
    void addSensorReading( const base::samples::DistanceImage& distImage, const Eigen::Affine3d& sensor2body )
    {
	// configure the processing chain for the distance image
	assert( currentBodyFrame );

	distFrame->setTransform( sensor2body );
	if( !distGrid )
	{
	    // create new grid using the parameters from the distance image
	    distGrid = new envire::DistanceGrid( distImage );

	    // distGrid has just been created and needs to be attached
	    distOp->addInput( distGrid.get() );
	    distGrid->setFrameNode( distFrame.get() );
	}
	distGrid->copyFromDistanceImage( distImage );

	envire::Pointcloud::Ptr distPc = new envire::Pointcloud();
	env->setFrameNode( distPc.get(), currentBodyFrame.get() );
	distPc->setLabel("dense");
	distOp->removeOutputs();
	distOp->addOutput( distPc.get() );

	distOp->updateAll();

	/*
	envire::GraphViz gv;
	gv.writeToFile( env, "/tmp/gv.dot" );
	*/
    }	

    /** adds a sensor reading for a feature array to an initialized node
     */
    void addSensorReading( const stereo::StereoFeatureArray& featureArray, const Eigen::Affine3d& sensor2body )
    {
	// configure the processing chain for the feature image 
	assert( currentBodyFrame );
	featureFrame->setTransform( sensor2body );
	// copy only up to a certain distance to omit the features further out
	featureArray.copyTo( *featurecloud.get(), 5.0 );

	envire::Featurecloud::Ptr featurePc = new envire::Featurecloud();
	env->setFrameNode( featurePc.get(), currentBodyFrame.get() );
	featurePc->setLabel("sparse");

	featurePc->copyFrom( featurecloud.get() );
	featurePc->itemModified();
    }

    /** adds an initialized node, with optional sensor readings to the node graph
     *
     * requires a previous call to initNode(), as well addSensorReading() calls
     * for each sensor reading.
     */
    void addNode()
    {
	assert( currentBodyFrame );

	// find relevant associations based on the current state of the graph
	// for now, we just associate with the previous node. 
	//if( prevBodyFrame )
	//{
	//    associateNodes( prevBodyFrame.get(), currentBodyFrame.get() );
	//}

	// perform association with other nodes, to actually form graphs!
	for( AISNavigation::Graph::VertexIDMap::iterator it = optimizer->vertices().begin(); 
		it != optimizer->vertices().end(); it++ )
	{
	    // don't associate with self
	    if( currentBodyFrame->getUniqueId() != it->first )
	    {
		envire::FrameNode::Ptr fn = env->getItem<envire::FrameNode>( it->first ).get();
		std::cout << "associate node " << fn->getUniqueId() << " with " << currentBodyFrame->getUniqueId() << "... ";
		bool result = associateNodes( fn.get(), currentBodyFrame.get() ); 
		std::cout << (result ? "match" : "no match") << std::endl;
	    }
	}

	// perform the graph optimization
	const int iterations = 5;
	optimizer->optimize( iterations, true );

	// write the poses back to the environment
	// for now, write all the poses back, could add an updated flag
	for( AISNavigation::Graph::VertexIDMap::iterator it = optimizer->vertices().begin(); 
		it != optimizer->vertices().end(); it++ )
	{
	    const int id = it->first;
	    const AISNavigation::PoseGraph3D::Vertex* vertex = static_cast<AISNavigation::PoseGraph3D::Vertex*>(it->second);

	    // get pose with uncertainty from Hogman
	    envire::TransformWithUncertainty tu( 
		    hogman2Eigen( vertex->transformation ), 
		    hogmanCov2EnvireCov( vertex->covariance ) );

	    // and set it in envire for the frameNode with the corresponding id
	    envire::FrameNode::Ptr fn = env->getItem<envire::FrameNode>( id ).get();
	    fn->setTransform( tu );

	    // update the bounds 
	    // TODO this could be optimized, as it may be to expensive to 
	    // update the bounds everytime we have a small change in position
	    getSensorMaps( fn.get() )->updateBounds();
	}

	// TODO see if we need to reassociate here 
	
	bodyFrame->setTransform( currentBodyFrame->getTransform() );
	prevBodyFrame = currentBodyFrame;
	currentBodyFrame = NULL;
    }

    /** will return a sensormaps structure for a given 
     * framenode. creates a new one, if not already existing.
     */
    SensorMaps* getSensorMaps( envire::FrameNode* fn )
    {
	const long id = fn->getUniqueId();

	// see if we can return a cached object
	std::map<long, SensorMaps*>::iterator 
	    f = nodeMap.find( id );

	if( f != nodeMap.end() )
	    return f->second;

	// otherwise create a new node
	SensorMaps* sm = new SensorMaps( fn );
	nodeMap.insert( make_pair( id, sm ) );

	// call update bounds once, so we have an initial
	// idea of the bounds
	sm->updateBounds();

	return sm;
    }

    /** helper struct that caches some information on the structure of the map
     * graph and associated information (e.g. bounding box).
     */
    struct SensorMaps
    {
	explicit SensorMaps( envire::FrameNode* a )
	    : stereoMap(NULL), sparseMap(NULL), frameNode( a )
	{
	    // go through all the maps in the framenode and see if they fit a sensor map 
	    std::list<envire::CartesianMap*> la = a->getMaps();
	    for( std::list<envire::CartesianMap*>::iterator it = la.begin();
		    it != la.end(); it ++ )
	    {
		if( (*it)->getLabel() == "dense" )
		    stereoMap = dynamic_cast<envire::Pointcloud*>( *it );
		else if( (*it)->getLabel() == "sparse" )
		    sparseMap = dynamic_cast<envire::Featurecloud*>( *it );
	    }
	}

	// update the bounds of the map using the uncertainty 
	// associated with the framenode 
	void updateBounds( double sigma = 3.0 )
	{
	    // get the extents from the individual maps first
	    if( extents.isEmpty() )
	    {
		// cache the extends, since they won't change 
		// locally
		if( stereoMap )
		    extents.extend( stereoMap->getExtents() );
		if( sparseMap )
		    extents.extend( sparseMap->getExtents() );
	    }

	    // the strategy is now to take the corner points, 
	    // and transform them including the uncertainty 
	    // provided. These points should all be included
	    // in the final bounds and should roughly provide
	    // the bounding box for the map including uncertainty

	    // reset the bounds
	    bounds.setEmpty();

	    // go through all 8 corners of the extents
	    for( int i=0; i<8; i++ )
	    {
		Eigen::Vector3d corner;
		for( int j=0; j<3; j++ )
		    corner[j] = (i>>j)&1 ? extents.min()[j] : extents.max()[j];

		// Transform the points with uncertainty
		envire::PointWithUncertainty uncertain_corner = 
		    frameNode->getTransformWithUncertainty() * envire::PointWithUncertainty(corner);

		// do a cholesky decomposition of the covariance matrix
		// in order to get the sigma points
		Eigen::LLT<Eigen::Matrix3d> llt;
		llt.compute( uncertain_corner.getCovariance() );
		Eigen::Matrix3d sigma_points = llt.matrixL();

		// extend the bounds by the sigma points
		for( int j=0; j<3; j++ )
		{
		    bounds.extend( 
			    uncertain_corner.getPoint() + sigma_points.col(j) );
		    bounds.extend( 
			    uncertain_corner.getPoint() - sigma_points.col(j) );
		}
	    }
	}

	envire::Pointcloud *stereoMap;
	envire::Featurecloud *sparseMap;
	// add more maps that can be associated here
	
	// store the frameNode pointer
	envire::FrameNode *frameNode;
	
	// cached local bounds
	Eigen::AlignedBox<double, 3> extents;
	
	/// The bounding box of the maps in global frame 
	/// including uncertainty
	Eigen::AlignedBox<double, 3> bounds;
    };

    /** 
     * associate two framenodes, if they are within a feasable distance
     * between each other and have overlapping bounding boxes.
     *
     * @return true if an association has been added
     */ 
    bool associateNodes( envire::FrameNode* a, envire::FrameNode* b )
    {
	// discard if distance between is too high
	// TODO: this is potentially dangerous as it doesn't take the
	// uncertainty into account... see how to make this safer, but still
	// fast.
	if( (a->getTransform().translation() - b->getTransform().translation()).norm() > max_node_radius )
	    return false;

	// get the sensor map objects for both frameNodes
	SensorMaps 
	    *sma = getSensorMaps( a ), 
	    *smb = getSensorMaps( b );

	// check if the bounding boxes have a common intersection
	// and return false if not
	if( sma->bounds.intersection( smb->bounds ).isEmpty() )
	    return false;

	// call the individual association methods
	if( sma->sparseMap && smb->sparseMap )
	{
	    if( associateSparseMap( sma->sparseMap, smb->sparseMap ) >= min_sparse_correspondences )
	    {
		if( sma->stereoMap && smb->stereoMap )
		    associateStereoMap( sma->stereoMap, smb->stereoMap );

		return true;
	    }
	    else
		return false;
	}
	else if( sma->stereoMap && smb->stereoMap )
	{
	    associateStereoMap( sma->stereoMap, smb->stereoMap );
	    return true;
	}

	// TODO store both successful and unsuccessful associations
	// since it doesn't make sense to try to associate twice
	return true;
    }

    void associateStereoMap( envire::Pointcloud* pc1, envire::Pointcloud* pc2 ) 
    {
	// perform an icp fitting of the two pointclouds
	// given a currently static parameter set
	envire::icp::ICPConfiguration conf;
	conf.model_density = 0.01;
	conf.measurement_density = 0.01;
	conf.max_iterations = 10;
	conf.min_mse = 0.005;
	conf.min_mse_diff = 0.0001;
	conf.overlap = 0.7;

	// use the envire ICP implementation
	// could think about using the PCL registration framework
	envire::icp::TrimmedKD icp;
	icp.addToModel( envire::icp::PointcloudAdapter( pc1, conf.model_density ) );
	icp.align( envire::icp::PointcloudAdapter( pc2, conf.measurement_density ),  
		conf.max_iterations, conf.min_mse, conf.min_mse_diff, conf.overlap );

	// come up with a covariance here
	// TODO replace with calculated covariance values 
	const double trans_error = 0.1;
	const double rot_error = 10.0/180.0*M_PI;
	
	Eigen::Matrix<double,6,1> cov_diag;
	cov_diag << Eigen::Vector3d::Ones() * rot_error, 
		 Eigen::Vector3d::Ones() * trans_error;

	Eigen::Matrix<double,6,6> cov = 
	    cov_diag.array().square().matrix().asDiagonal();

	Eigen::Affine3d bodyBtoBodyA = 
	    pc2->getFrameNode()->relativeTransform( pc1->getFrameNode() );

	// add the egde to the optimization framework 
	// this will update an existing edge
	optimizer->addEdge( 
		optimizer->vertex( pc1->getFrameNode()->getUniqueId() ),
		optimizer->vertex( pc2->getFrameNode()->getUniqueId() ),
		eigen2Hogman( bodyBtoBodyA ),
		envireCov2HogmanInf( cov )
		);
    }

    /** 
     * try to associate two sparse feature clouds.
     *
     * @return the number of matching interframe features. This can be used as a measure of quality
     * for the match.
     */
    size_t associateSparseMap( envire::Featurecloud *fc1, envire::Featurecloud *fc2 )
    {
	stereo::StereoFeatures f;
	stereo::FeatureConfiguration config;
	config.isometryFilterMaxSteps = 1000;
	f.setConfiguration( config );

	f.calculateInterFrameCorrespondences( fc1, fc2, stereo::FILTER_ISOMETRY );
	std::vector<std::pair<long, long> > matches = f.getInterFrameCorrespondences();

	if( f.getInterFrameCorrespondences().size() >= min_sparse_correspondences )
	{
	    Eigen::Affine3d bodyBtoBodyA = f.getInterFrameCorrespondenceTransform();

	    // come up with a covariance here
	    // TODO replace with calculated covariance values 
	    const double trans_error = 0.1;
	    const double rot_error = 10.0/180.0*M_PI;

	    Eigen::Matrix<double,6,1> cov_diag;
	    cov_diag << Eigen::Vector3d::Ones() * rot_error, 
		     Eigen::Vector3d::Ones() * trans_error;

	    Eigen::Matrix<double,6,6> cov = 
		cov_diag.array().square().matrix().asDiagonal();

	    // add the egde to the optimization framework 
	    // this will update an existing edge
	    optimizer->addEdge( 
		    optimizer->vertex( fc1->getFrameNode()->getUniqueId() ),
		    optimizer->vertex( fc2->getFrameNode()->getUniqueId() ),
		    eigen2Hogman( bodyBtoBodyA ),
		    envireCov2HogmanInf( cov )
		    );
	}

	return f.getInterFrameCorrespondences().size();
    }

};
}

#endif
