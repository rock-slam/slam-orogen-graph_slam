#ifndef __GRAPH_SLAM_POSE_GRAPH_HPP__
#define __GRAPH_SLAM_POSE_GRAPH_HPP__

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/Featurecloud.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/operators/DistanceGridToPointcloud.hpp>

#include <aislib/graph_optimizer/graph_optimizer3d_hchol.h>
#include <envire/icp.hpp>
#include <envire/icpConfigurationTypes.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

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
    t << eigen_matrix.bottomRightCorner<3,3>(), eigen_matrix.topRightCorner<3,3>(),
      eigen_matrix.bottomLeftCorner<3,3>(), eigen_matrix.topLeftCorner<3,3>();

    return eigen2Hogman( Eigen::Matrix<double,6,6>(t.inverse()) );
}


class PoseGraph
{
    envire::Environment *env;
    AISNavigation::GraphOptimizer3D *optimizer;
    std::map<envire::FrameNode*, AISNavigation::PoseGraph3D::Vertex*> nodeMap;

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
	: env( env ), optimizer( new AISNavigation::HCholOptimizer3D( num_levels, node_distance ) ) 
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

	envire::GraphViz gv;
	gv.writeToFile( env, "/tmp/gv.dot" );
    }	

    /** adds a sensor reading for a feature array to an initialized node
     */
    void addSensorReading( const stereo::StereoFeatureArray& featureArray, const Eigen::Affine3d& sensor2body )
    {
	// configure the processing chain for the feature image 
	assert( currentBodyFrame );
	featureFrame->setTransform( sensor2body );
	featureArray.copyTo( *featurecloud.get() );

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
	// TODO perform association with other nodes, to actually form graphs!
	
	if( prevBodyFrame )
	{
	    associateNodes( prevBodyFrame.get(), currentBodyFrame.get() );
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

	    envire::FrameNode::Ptr fn = env->getItem<envire::FrameNode>( id ).get();
	    fn->setTransform( hogman2Eigen( vertex->transformation ) );
	}

	// TODO see if we need to reassociate here 
	
	bodyFrame->setTransform( currentBodyFrame->getTransform() );
	prevBodyFrame = currentBodyFrame;
	currentBodyFrame = NULL;
    }

    struct SensorMaps
    {
	explicit SensorMaps( envire::FrameNode* a )
	    : stereoMap(NULL)
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

	envire::Pointcloud *stereoMap;
	envire::Featurecloud *sparseMap;
	// add more maps that can be associated here
    };

    void associateNodes( envire::FrameNode* a, envire::FrameNode* b )
    {
	// get the sensor map objects for both frameNodes
	// (could be cached later)
	SensorMaps sma( a ), smb( b );

	// call the individual association methods
	if( sma.stereoMap && smb.stereoMap )
	//    associateStereoMap( sma.stereoMap, smb.stereoMap );
	;

	if( sma.sparseMap && smb.sparseMap )
	    associateSparseMap( sma.sparseMap, smb.sparseMap );
	
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

	Eigen::Affine3d bodyAtoBodyB = 
	    pc2->getFrameNode()->relativeTransform( pc1->getFrameNode() );

	// add the egde to the optimization framework 
	// this will update an existing edge
	optimizer->addEdge( 
		optimizer->vertex( pc1->getFrameNode()->getUniqueId() ),
		optimizer->vertex( pc2->getFrameNode()->getUniqueId() ),
		eigen2Hogman( bodyAtoBodyB ),
		envireCov2HogmanInf( cov )
		);
    }

    void associateSparseMap( envire::Featurecloud *fc1, envire::Featurecloud *fc2 )
    {
	// get features from array
	cv::Mat feat1 = cv::Mat( fc1->size(), fc1->descriptorSize, cv::DataType<float>::type, &fc1->descriptors[0]); 
	cv::Mat feat2 = cv::Mat( fc2->size(), fc2->descriptorSize, cv::DataType<float>::type, &fc2->descriptors[0]); 

	// match the features
	const int knn = 1;
	std::vector<std::vector<cv::DMatch> > matches; 
	cv::FlannBasedMatcher matcher;
	matcher.knnMatch( feat1, feat2, matches, knn );

	// find the transformation
	envire::icp::Pairs pairs;
	for( size_t i=0; i<matches.size(); i++ )
	{
	    const cv::DMatch &match( matches[i].front() );
	    pairs.add( 
		    fc1->vertices[match.trainIdx], 
		    fc2->vertices[match.queryIdx], 
		    match.distance );
	}
	pairs.trim( pairs.size() * 0.3 );

	Eigen::Affine3d bodyAtoBodyB = pairs.getTransform();
	
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
		eigen2Hogman( bodyAtoBodyB ),
		envireCov2HogmanInf( cov )
		);
    }

    ~PoseGraph()
    {
	delete optimizer;
    }

};
}

#endif
