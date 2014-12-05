#include "main.h"

/*
 * TESTING OpenCV: SURF matching
 */
void getTransformationMatrix(Mat currImage) {
	vector<KeyPoint> keypoints_prev_frame, keypoints_curr_frame;
  	Mat descriptors_prev_frame, descriptors_curr_frame;
	vector< DMatch > matches;

	detector.detect( prevImage, keypoints_prev_frame );
	detector.detect( currImage, keypoints_curr_frame );

  	extractor.compute( prevImage, keypoints_prev_frame, descriptors_prev_frame );
  	extractor.compute( currImage, keypoints_curr_frame, descriptors_curr_frame );

  	matcher.match( descriptors_prev_frame, descriptors_curr_frame, matches );

	double max_dist = 0; double min_dist = 100;
	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < matches.size(); i++ ) { 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	vector< DMatch > good_matches;

	for( int i = 0; i < matches.size(); i++ ) { 
		if( matches[i].distance < max(2*min_dist, 0.02) ) { 
			good_matches.push_back( matches[i]);
		}
	}
	//-- Localize the object
	vector<Point2f> prevFrame;
	vector<Point2f> currFrame;

	for( int i = 0; i < good_matches.size(); i++ ) {
		//-- Get the keypoints from the good matches
		prevFrame.push_back( keypoints_prev_frame[ good_matches[i].queryIdx ].pt );
		currFrame.push_back( keypoints_curr_frame[ good_matches[i].trainIdx ].pt );
	}

	Mat H = findHomography( prevFrame, currFrame, CV_RANSAC );
	cout << "Transformation Matrix = "<< endl << " "  << H << endl << endl;
}

/*
 * TESTING OpenGL
 * Simple method to draw simple polygons 
 */
void displayPolygon() {

  	// Set every pixel in the frame buffer to the current clear color.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();//load identity matrix
	glDisable(GL_TEXTURE_2D);

    //glTranslatef(0.0f,0.0f,-4.0f);//move forward 4 units
    
    glColor3f(0.0f,0.0f,1.0f); //blue color
        
    glBegin(GL_POLYGON);//begin drawing of polygon
      glVertex3f(-0.5f,0.5f,0.0f);//first vertex
      glVertex3f(0.5f,0.5f,0.0f);//second vertex
      glVertex3f(1.0f,0.0f,0.0f);//third vertex
      glVertex3f(0.5f,-0.5f,0.0f);//fourth vertex
      glVertex3f(-0.5f,-0.5f,0.0f);//fifth vertex
      glVertex3f(-1.0f,0.0f,0.0f);//sixth vertex
    glEnd();//end drawing of polygon

    /*
    glBegin(GL_POLYGON);//begin drawing of polygon
      //glVertex3f(-0.5f,0.5f,0.0f);//first vertex
      //glVertex3f(0.5f,0.5f,0.0f);//second vertex
      glVertex3f(320,0,0.0f);//third vertex
      glVertex3f(640,240,0.0f);//fourth vertex
      glVertex3f(320,480,0.0f);//fifth vertex
      glVertex3f(0,240,0.0f);//sixth vertex
    glEnd();//end drawing of polygon
    */
	glutSwapBuffers();
}


/*
 * Rotates the camera to show 3D points
 */
void rotateCamera() {
	static double angle = 0.;
	static double radius = 3.;
	double x = radius*sin(angle);
	double z = radius*(1-cos(angle)) - radius/2;
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	gluLookAt(x,0,z,0,0,radius/2,0,-1,0);
	angle += 0.05;
}

void changeView() {
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	gluLookAt(eyeX,eyeY,eyeZ,eyeX + dx,eyeY + dy,eyeZ+dz,0,-1,0);
}

/*
 * Define Kinect's keyboard controls
 * glutKeyboardFunc Handler
 */
void keyPressed(unsigned char key, int x, int y) {
	if (key == 27) {
		device->setLed(LED_OFF);
		freenect_angle = 0;
		device->setTiltDegrees(freenect_angle);
		glutDestroyWindow(window);
	}
	if (key == '1') {
		device->setLed(LED_GREEN);
	}
	if (key == '2') {
		device->setLed(LED_RED);
	}
	if (key == '3') {
		device->setLed(LED_YELLOW);
	}
	if (key == '4') {
		device->setLed(LED_BLINK_GREEN);
	}
	if (key == '5') {
		device->setLed(LED_BLINK_RED_YELLOW);
	}
	if (key == '0') {
		device->setLed(LED_OFF);
	}
	if (key == 'd') {
		device->setDepthFormat(requested_depth_format);
	}
	if (key == 'f') {
		if (requested_video_format == FREENECT_VIDEO_IR_8BIT) {
			requested_video_format = FREENECT_VIDEO_RGB;
		} else if (requested_video_format == FREENECT_VIDEO_RGB){
			requested_video_format = FREENECT_VIDEO_YUV_RGB;
		} else {
			requested_video_format = FREENECT_VIDEO_IR_8BIT;
		}
		device->setVideoFormat(requested_video_format);
	}

	if (key == 'w') {
		freenect_angle++;
		if (freenect_angle > 30) {
			freenect_angle = 30;
		}
		device->setTiltDegrees(freenect_angle);
	}
	if (key == 'S') {
		savePCD_();
	}
	if (key =='L') {
		loadPCD_();
	}
	if (key == 'C') {
		setPointCloudColor(cloud_final, 255, 0, 0);
	} 
	if (key == 'x') {
		freenect_angle--;
		if (freenect_angle < -30) {
			freenect_angle = -30;
		}
		device->setTiltDegrees(freenect_angle);
	}
	if (key == 'e') {
		freenect_angle = 10;
		device->setTiltDegrees(freenect_angle);
	}
	if (key == 'c') {
		freenect_angle = -10;
		device->setTiltDegrees(freenect_angle);
	}
	if (key == 'v') {
		register3DScene();
		registerScene = 1;
		showScene = 1;
	}
	if (key == 'j') {
		angle -= 0.1f;
		dx = sin(angle);
		dz = cos(angle);
	}
	if (key == 'J') {
		eyeX -= dx*.05f;
		eyeZ -= dz*.05f;
	}
	if (key == 'n') {
		angle += 0.1f;
		dy= sin(angle);
		dz = cos(angle);
	}
	if (key == 'N') {
		angle -= 0.1f;
		dy = sin(angle);
		dz = cos(angle);
	}
	if (key == 'k') {
		angle += 0.1f;
		dx = sin(angle);
		dz = cos(angle);
	}
	if (key == 'K') {
		eyeX += dx*.05f;
		eyeZ += dz*.05f;
	}
	if (key == 'i') {
		eyeX -= dz*.05f;
		eyeZ += dx*.05f;
	}
	if (key == 'I') {
		eyeX += dz*.05f;
		eyeZ -= dx*.05f;
	}
	if (key == 'm') {
		eyeY -= dz*.05f;
		eyeZ += dy*.05f;
	}
	if (key == 'M') {
		eyeY += dz*.1f;
		eyeZ -= dy*.1f;
	}
}



//show the current kinect RGB scene
void drawRGBScene() {
	//static std::vector<uint8_t> depth(640*480*4);
	static std::vector<uint8_t> rgb(640*480*4);
	device->updateState();

	printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f", freenect_angle, device->getState().getTiltDegs());
	fflush(stdout);

	//bool depth_registered = false;
	bool rgb_registered = false;

	while(!rgb_registered)
		rgb_registered = device->getRGB(rgb);

	got_frames = 0;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	//glViewport(0, (float)HEIGHT/2, (float)WIDTH/2, HEIGHT);
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 4, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &rgb[0]);

	glBegin(GL_TRIANGLE_FAN);
		glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
		glTexCoord2f(0, 0); glVertex3f(0,0,0);
		glTexCoord2f(1, 0); glVertex3f(640,0,0);
		glTexCoord2f(1, 1); glVertex3f(640,480,0);
		glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();

	glutSwapBuffers();
}

//show the current kinect depth scene
void drawDepthScene() {
	static std::vector<uint8_t> depth(640*480*4);

	device->updateState();
	printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f", freenect_angle, device->getState().getTiltDegs());
	fflush(stdout);

	bool depth_registered = false;

	while(!depth_registered)
		depth_registered = device->getDepth(depth);

	got_frames = 0;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	//glViewport(0, (float)HEIGHT/2, (float)WIDTH/2, HEIGHT);
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 4, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &depth[0]);

	glBegin(GL_TRIANGLE_FAN);
		glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
		glTexCoord2f(0, 0); glVertex3f(0,0,0);
		glTexCoord2f(1, 0); glVertex3f(640,0,0);
		glTexCoord2f(1, 1); glVertex3f(640,480,0);
		glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();

	glutSwapBuffers();
}

void setPointCloudColor(PointCloud<PointXYZRGB>::Ptr pc,
						int r,
						int g,
						int b) {

	for(size_t i = 0; i < pc->points.size(); i++) {
		pc->points[i].r = r;
		pc->points[i].g = g;
		pc->points[i].b = b;
	}
}

/*
 * Doesn't work very well... :(
 */
void filterCorrespondences(PointCloud<PointXYZI>::Ptr src_pts,
                           PointCloud<PointXYZI>::Ptr tar_pts,
                           vector<int>& src2tar, 
                           vector<int>& tar2src,
                           CorrespondencesPtr cors) {

    cout << "Begin evaluating and filtering correspondences.." << endl;
    vector<pair<unsigned, unsigned> > correspondences;
    for (unsigned cIdx = 0; cIdx < src2tar.size(); ++cIdx) {
        if (tar2src[src2tar[cIdx]] == cIdx) {
            correspondences.push_back(make_pair(cIdx, src2tar[cIdx]));
        }
    }

    cors->resize(correspondences.size());
    for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx) {
        (*cors)[cIdx].index_query = correspondences[cIdx].first;
        (*cors)[cIdx].index_match = correspondences[cIdx].second;
    }

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZI> rejector;
    rejector.setInputCloud(src_pts);
    rejector.setTargetCloud(tar_pts);
    rejector.setMaxIterations(500);
    rejector.setInlierThreshold(0.1f);
    rejector.setInputCorrespondences(cors);
    rejector.getCorrespondences(*cors);
    
    cout << "Correspondences filtered: " << correspondences.size() << endl;
}

void filterPointCloud(PointCloud<PointXYZRGB>::Ptr in_cloud, 
					  float leaf_size,
            		  PointCloud<PointXYZRGB>::Ptr out_cloud) {

	cout << "\nBegin downsampling point-cloud..." << endl;
	cout << "Initial point count: " << in_cloud->points.size() << endl;

	VoxelGrid<PointXYZRGB> voxel_grid;
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	voxel_grid.setInputCloud(in_cloud);
	voxel_grid.filter(*out_cloud);
	
	cout << "\nFinished downsampling point-cloud !" << endl;
	cout << "Filtered point cloud: " << out_cloud->points.size() << endl;
}

Matrix4f transformationEstimation(PointCloud<PointXYZI>::Ptr src_pts,
       						      PointCloud<PointXYZI>::Ptr tar_pts,
        						  CorrespondencesPtr cors) {

	cout << "Begin transformation estimation" << endl;

    Matrix4f transformationMtx = Matrix4f::Identity();
    pcl::registration::TransformationEstimation<PointXYZI, PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<PointXYZI, PointXYZI>);
    transformation_estimation->estimateRigidTransformation(*src_pts, *tar_pts, *cors, transformationMtx);
    
	cout << "Transformation:\n" << transformationMtx << endl;    
    return transformationMtx;
}

Matrix4f RANSACRGBalign(PointCloud<PointXYZRGB>::Ptr src_cloud,
				     PointCloud<PointXYZRGB>::Ptr tar_cloud,
				 	 PointCloud<FPFHSignature33>::Ptr src_desc, 
				 	 PointCloud<FPFHSignature33>::Ptr tar_desc) {

    PointCloud<PointXYZRGB>::Ptr src2tar (new PointCloud<PointXYZRGB>);
    time_t start;
    time_t end;
    
    //start the timer
    time(&start);

	SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> reg;
	reg.setMinSampleDistance(SAC_MIN_SAMPLE);
	reg.setMaxCorrespondenceDistance(SAC_MAX_COR_DIST);
	reg.setMaximumIterations(SAC_MAX_ITER);
	reg.setNumberOfSamples(3);

	reg.setInputCloud(src_cloud);
	reg.setInputTarget(tar_cloud);
	reg.setSourceFeatures(src_desc);
	reg.setTargetFeatures(tar_desc);

	reg.align(*src2tar);

  	//stop the timer
  	time(&end);
	double secs = difftime(end, start);
	cout << "\nRANSAC alignment took: " << secs << " secs" << endl;

	cout << "Has converged:" << reg.hasConverged() << endl;
	cout << "Fitness Score: " << reg.getFitnessScore() << endl;
	cout << "Transformation:\n" << reg.getFinalTransformation() << endl;
	cout << "RANSAC Finished";

	return reg.getFinalTransformation();
}

Matrix4f RANSACalign(PointCloud<PointXYZI>::Ptr src_cloud,
				     PointCloud<PointXYZI>::Ptr tar_cloud,
				 	 PointCloud<FPFHSignature33>::Ptr src_desc, 
				 	 PointCloud<FPFHSignature33>::Ptr tar_desc) {

    PointCloud<PointXYZI>::Ptr src2tar (new PointCloud<PointXYZI>);
    time_t start;
    time_t end;
    
    //start the timer
    time(&start);

	SampleConsensusInitialAlignment<PointXYZI, PointXYZI, FPFHSignature33> reg;
	reg.setMinSampleDistance(SAC_MIN_SAMPLE);
	reg.setMaxCorrespondenceDistance(SAC_MAX_COR_DIST);
	reg.setMaximumIterations(SAC_MAX_ITER);
	reg.setNumberOfSamples(3);

	reg.setInputCloud(src_cloud);
	reg.setInputTarget(tar_cloud);
	reg.setSourceFeatures(src_desc);
	reg.setTargetFeatures(tar_desc);

	reg.align(*src2tar);

  	//stop the timer
  	time(&end);
	double secs = difftime(end, start);
	cout << "\nRANSAC alignment took: " << secs << " secs" << endl;

	cout << "Has converged:" << reg.hasConverged() << endl;
	cout << "Fitness Score: " << reg.getFitnessScore() << endl;
	cout << "Transformation:\n" << reg.getFinalTransformation() << endl;
	cout << "RANSAC Finished";

	return reg.getFinalTransformation();
}

Matrix4f ICPalign(PointCloud<PointXYZRGB>::Ptr src_cloud, 
				  PointCloud<PointXYZRGB>::Ptr tar_cloud,
				  PointCloud<PointXYZRGB>::Ptr src2tar,
				  Matrix4f guess) {

	//PointCloud<PointXYZRGB>::Ptr src2tar (new PointCloud<PointXYZRGB>);
    time_t start;
    time_t end;
    
    //start the timer
    time(&start);

	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setInputCloud(src_cloud);
	icp.setInputTarget(tar_cloud);

	// Set the max correspondence distance - correspondences with higher distances will be ignored
	icp.setMaxCorrespondenceDistance(0.0001f);
	// Set the maximum number of iterations
	icp.setMaximumIterations(300);
	// Set the transformation epsilon
	icp.setTransformationEpsilon(1e-8);
	// Set the euclidean distance difference epsilon
	icp.setEuclideanFitnessEpsilon(0.0001f);
 	//icp.setRANSACOutlierRejectionThreshold(0.001);

  	icp.align(*src2tar, guess);

  	//stop the timer
  	time(&end);
	double secs = difftime(end, start);
	cout << "\nICP registration took: " << secs << " secs" << endl; 

  	cout << "ICP Aligned: " << icp.hasConverged() << endl;
  	cout << "ICP Score: " << icp.getFitnessScore() << endl;
  	cout << icp.getFinalTransformation() << endl;

	return icp.getFinalTransformation();
}

void getInterestPoints(PointCloud<PointXYZRGB>::Ptr input_cloud, PointCloud<PointXYZI>::Ptr interest_points) {

    SIFTKeypoint<PointXYZRGB, PointXYZI>* sift3D = new SIFTKeypoint<PointXYZRGB, PointXYZI>;
    pcl::search::KdTree<PointXYZRGB>::Ptr treeKD (new pcl::search::KdTree<PointXYZRGB>());

    cout << "Computing SIFT points..." << endl;
    sift3D->setScales(SIFT_SCALE, 6, 10);
    sift3D->setMinimumContrast(0.5f);
    sift3D->setSearchMethod(treeKD);
    sift3D->setInputCloud(input_cloud);
    sift3D->compute(*interest_points);
    cout << "SIFT points computed: " << interest_points->points.size() << endl;
}

void getRGBPointDescriptors(PointCloud<PointXYZRGB>::Ptr ipts, PointCloud<FPFHSignature33>::Ptr feature_descriptors) {
	cout << "Obtaining descriptors for interest points..." << endl;
	//Set
	FPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33>::Ptr fpfh (new FPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33>);
    fpfh->setSearchMethod(pcl::search::Search<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
    fpfh->setRadiusSearch(FEATURE_RADIUS);
    fpfh->setInputCloud(ipts);

	PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    NormalEstimation<PointXYZRGB, Normal> normal_estimation;
    normal_estimation.setSearchMethod(pcl::search::Search<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
    normal_estimation.setRadiusSearch(NORMAL_RADIUS);
    normal_estimation.setInputCloud(ipts);
    normal_estimation.compute(*normals);

    fpfh->setInputNormals(normals);
    fpfh->compute(*feature_descriptors);

    cout << "Feature descriptors obtained: " << feature_descriptors->points.size() << endl;
}


void getPointDescriptors(PointCloud<PointXYZI>::Ptr interest_points, PointCloud<FPFHSignature33>::Ptr feature_descriptors) {
	cout << "Obtaining descriptors for interest points..." << endl;
	//Set
    PointCloud<PointXYZRGB>::Ptr ipts (new PointCloud<PointXYZRGB>);
	copyPointCloud(*interest_points, *ipts);

	FPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33>::Ptr fpfh (new FPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33>);
    //FPFHEstimation<PointXYZRGB, Normal, FPFHSignature33>::Ptr fpfh;
    fpfh->setSearchMethod(pcl::search::Search<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
    fpfh->setRadiusSearch(FEATURE_RADIUS);
    fpfh->setInputCloud(ipts);

	PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    NormalEstimation<PointXYZRGB, Normal> norm_est;
    norm_est.setSearchMethod(pcl::search::Search<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
    norm_est.setRadiusSearch(NORMAL_RADIUS);
    norm_est.setInputCloud(ipts);
    norm_est.compute(*normals);

    fpfh->setInputNormals(normals);
    fpfh->compute(*feature_descriptors);

    cout << "Feature descriptors obtained: " << feature_descriptors->points.size() << endl;
}

void getCorrespondance(PointCloud<FPFHSignature33>::Ptr src, PointCloud<FPFHSignature33>::Ptr tar, vector<int>& correspondences) {
	cout << "Iterating through descriptors and finding correspondences" << endl;
	correspondences.resize(src->size());

	KdTreeFLANN<FPFHSignature33> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(tar);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    vector<int> k_indices(k);
    vector<float> k_squared_distances(k);
    for (size_t i = 0; i < src->size(); i++) {
        descriptor_kdtree.nearestKSearch (*src, i, k, k_indices, k_squared_distances);
        correspondences[i] = k_indices[0];
    }
    cout << "Feature Correspondences found: " << correspondences.size() << endl;
}

// Registers 3D depth points from the Kinect
void register3DScene() {
	PointCloud<PointXYZRGB>::Ptr cloud_in (new PointCloud<PointXYZRGB>);
	bool depth_registered = false;
	bool rgb_registered = false;

	while(!depth_registered)
		depth_registered = device->getDepthInMM(sceneDepth);
	while(!rgb_registered)
		rgb_registered = device->getRGB(sceneRGB);

	cloud_in->width    = 640;
	cloud_in->height   = 480;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);

	int cInx = 0;
	int largestZ = 0;
	for(size_t i=0; i < FREENECT_FRAME_PIX; i++) {
    		float z = (float)sceneDepth[i]/10;
    		float ii = i % FREENECT_FRAME_W;
    		float j = floor(i/FREENECT_FRAME_W);
    		// range of points in the point cloud
    		if(z > 0 && z < MAX_Z) {
    			largestZ = z > largestZ ? z : largestZ;
	    		float x = (ii - FREENECT_FRAME_W/2) * (z - 10) * .0021;
	    		float y = (j - FREENECT_FRAME_H/2) * (z - 10) * .0021;
	    		cloud_in->points[cInx].x = (float)x/100;
	    		cloud_in->points[cInx].y = (float)y/100;
	    		cloud_in->points[cInx].z = (float)z/100;
	    		cloud_in->points[cInx].r = sceneRGB[i*3];
	    		cloud_in->points[cInx].g = sceneRGB[i*3+1];
	    		cloud_in->points[cInx].b = sceneRGB[i*3+2];
	    		cInx++;
  			}
	}

	if(cloud_prev->points.size() == 0) {
    	filterPointCloud(cloud_in, LEAF_SIZE, cloud_prev);
		//*cloud_final = *cloud_prev;
		copyPointCloud(*cloud_prev, *cloud_final);
		number_of_points = cloud_final->points.size();
	}else {
		//ICPalign(cloud_in, cloud_prev, cloud_final);
 	 	PointCloud<PointXYZI>::Ptr src_ipts       (new PointCloud<PointXYZI>);
    	PointCloud<PointXYZI>::Ptr tar_ipts       (new PointCloud<PointXYZI>);
    	PointCloud<FPFHSignature33>::Ptr src_desc (new PointCloud<FPFHSignature33>);
    	PointCloud<FPFHSignature33>::Ptr tar_desc (new PointCloud<FPFHSignature33>);

 	 	PointCloud<PointXYZRGB>::Ptr cloud_in_filtered  (new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr registered  (new PointCloud<PointXYZRGB>);

 	 	PointCloud<PointXYZRGB>::Ptr cloud_trans  (new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloud_trans_final  (new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloud_final_filtered  (new PointCloud<PointXYZRGB>);

		cout << "Leaf size is: " << LEAF_SIZE << endl;

		filterPointCloud(cloud_in, LEAF_SIZE, cloud_in_filtered);

		//Get interest points
    	getInterestPoints(cloud_in_filtered, src_ipts);
    	getInterestPoints(cloud_prev, tar_ipts);
		//copyPointCloud(*cloud_in_filtered, *src_ipts);
		//copyPointCloud(*cloud_prev, *tar_ipts);

		//getRGBPointDescriptors(cloud_in_filtered, src_desc);
		//getRGBPointDescriptors(cloud_prev, tar_desc);

    	//Get descriptors for interest points
    	getPointDescriptors(src_ipts, src_desc);
    	getPointDescriptors(tar_ipts, tar_desc);

	    //Find Correspondences
	    //vector<int> src2tar;
	    //vector<int> tar2src;
	    //getCorrespondance(src_desc, tar_desc, src2tar);
	    //getCorrespondance(tar_desc, src_desc, tar2src);

	    //Filter Correnspondences
	    //CorrespondencesPtr cors (new Correspondences);
	    //filterCorrespondences(src_ipts, tar_ipts, src2tar, tar2src, cors);

	    //Initial transformation
	    //Matrix4f Ti = transformationEstimation(src_ipts, tar_ipts, cors);
	    //Matrix4f Ti = RANSACRGBalign(cloud_in_filtered, cloud_prev, src_desc, tar_desc);
    	Matrix4f Ti = RANSACalign(src_ipts, tar_ipts, src_desc, tar_desc);
    	T_matrix = T_matrix * Ti;

    	pcl::transformPointCloud(*cloud_in_filtered, *cloud_trans, T_matrix);
    	cout << "Initial transformation complete!" << endl;
 	 	//cout << "Transformed cloud has: " << cloud_trans->points.size() << " points" << endl;
    	
    	//filterPointCloud(cloud_trans, 0.005, filtered_cloud_trans);
		//Matrix4f FTi = ICPalign(cloud_in_filtered, cloud_prev, registered, Ti);
    	//pcl::transformPointCloud(*cloud_in_filtered, *cloud_trans_final, FTi);
    	//T_matrix = T_matrix * FTi;
		//setPointCloudColor(cloud_trans, 255, 0, 0);
    	*cloud_final += *cloud_trans;

    	filterPointCloud(cloud_final, 0.005, cloud_final_filtered);
		copyPointCloud(*cloud_final_filtered, *cloud_final);
    	number_of_points = cloud_final->points.size();
		copyPointCloud(*cloud_in_filtered, *cloud_prev);
	}

	printf("\n Number of points: %d", number_of_points);
	printf("\n depth was registered: %d", depth_registered);
	printf("\n rgb was registered: %d", rgb_registered);
}

//Shows the axis alignment
void showAxis() {
	 //draw axis lines//
	 
	 //x-axis - RED
	 glBegin(GL_LINES);
	 glColor3f(1.0f,0.0f,0.0f);
	 glVertex3f(0.0f,0.0f,0.0f);
	 glVertex3f(100.0f, 0.0f,0.0f);
	 glEnd();

	 //y-axis - GREEN
	 glBegin(GL_LINES);
	 glColor3f(0.0f,1.0f,0.0f);
 	 glVertex3f(0.0f, 100.0f,0.0f);
	 glVertex3f(0.0f,0.0f,0.0f);
	 glEnd();

	 //z-axis - BLUE
	 glBegin(GL_LINES);
	 glColor3f(0.0f,0.0f,1.0f);
	 glVertex3f(0.0f,0.0f,0.0f);
	 glVertex3f(0.0f, 0.0f,100.0f);
	 glEnd();

	 glColor3f(1,1,1);
}

// Constructs 3D point cloud scene
void constructScene() {

		//Mat grayScaleImage(480, 640, CV_8UC1);
		//displayPolygon();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();
		//glViewport(0, 0, (float)WIDTH, HEIGHT/2);

	    //rotateCamera();
		changeView();

    	if(showScene) {
	    	glBegin(GL_POINTS);
		 //    for (int i = 0; i < FREENECT_FRAME_PIX; i++) {
		 //    		float z = (float)sceneDepth[i]/10;
		 //    		float ii = i % FREENECT_FRAME_W;
		 //    		float j = floor(i/FREENECT_FRAME_W);
		 //    		grayScaleImage.at<uchar>(j,ii) = (int)(.299*sceneRGB[i*3] + .587*sceneRGB[i*3+1] + .114*sceneRGB[i*3+2]);
		 //    		if(z > 0 && z < 1000) {
			//   	  		glColor3f((float)sceneRGB[i*3]/255, (float)sceneRGB[i*3+1]/255, (float)sceneRGB[i*3+2]/255);
			//     		float x = (ii - FREENECT_FRAME_W/2) * (z - 10) * .0021;
			//     		float y = (j - FREENECT_FRAME_H/2) * (z - 10) * .0021;
	  // 	  				glVertex3f(x/20, y/20, z/20);
	  // 	  			}
			// }
			for (int i = 0; i < cloud_final->points.size(); i++) {
				//glColor3f(1.0f, 1.0f, 1.0f);
				glColor3f((float)cloud_final->points[i].r/255, (float)cloud_final->points[i].g/255, (float)cloud_final->points[i].b/255);
				glVertex3f(cloud_final->points[i].x, cloud_final->points[i].y, cloud_final->points[i].z);
			}
		    glEnd();

		 //    if(registerScene) { 
			//     if (prevReg) {
			//     	getTransformationMatrix(grayScaleImage);
			//     	prevReg = 0;
			//     }
			//     prevImage = grayScaleImage;
			//     imshow("Gray Image", grayScaleImage);
			//     registerScene = 0;
			// }
			// prevReg = 1;
		}else {
			rotateCamera();
			showAxis();
		}

	    glutSwapBuffers();
}

/*
 * Initialize OpenGL variables for 3D perspective projection
 * Currently: used for botton "point cloud" window 
 * 
 */
void defaultGL() {
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
    //glMatrixMode(GL_MODELVIEW);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho (0, 640, 480, 0, 0.0f, 10000);
	gluPerspective(150.0f, (float)FREENECT_FRAME_W/FREENECT_FRAME_H, 0.1f, 100);
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	gluLookAt(0,0,0,0,0,1,0,-1,0);
}

/*
 * Initialize OpenGL variables for 2D texture drawings
 * Currently: used for top 2 windows
 */
void InitGL() {
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	//glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 640, 480, 0, 0.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

/*
 * Glut idle function
 * Redraws all of the windows regardless of window focus
 */
void idle() {
	int currentWindow = glutGetWindow();
	glutSetWindow(window);
	glutPostRedisplay();
	glutSetWindow(subWindow1);
	glutPostRedisplay();
	glutSetWindow(subWindow2);
	glutPostRedisplay();
	glutSetWindow(subWindow3);
	glutPostRedisplay();
	glutSetWindow(currentWindow);
}

// Main Window that carries the subwindows
void mainDisplay() {
	glClearColor(0.8, 0.8, 0.8, 0.0);  
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glutSwapBuffers();
}

/*
 * Sets up the GLUT display
 * Currently: 3 Windows
 */
void setUpDisplay(MyFreenectDevice* device){
	glutInit(&g_argc, g_argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutInitWindowPosition(INIT_POS_X, INIT_POS_Y);

	//Create the UI layout
	window = glutCreateWindow("Kinect 3D Scene");
		glutDisplayFunc(mainDisplay);
		glutKeyboardFunc(keyPressed);
		//glutDisplayFunc(&DrawGLScene);
		//glutIdleFunc(&DrawGLScene);
		defaultGL();
	subWindow1 = glutCreateSubWindow(window, BORDER, BORDER, WIDTH/2-3/2*BORDER, HEIGHT/2-3/2*BORDER);
		glutDisplayFunc(drawRGBScene);
		glutKeyboardFunc(keyPressed);
		//glutIdleFunc(&DrawGLScene);
		InitGL();
	subWindow2 = glutCreateSubWindow(window, WIDTH/2+BORDER/2, BORDER, WIDTH/2-3/2*BORDER, HEIGHT/2-3/2*BORDER);
		glutDisplayFunc(drawDepthScene);
		glutKeyboardFunc(keyPressed);
		//glutIdleFunc(&DrawGLScene);
		InitGL();
	subWindow3 = glutCreateSubWindow(window, BORDER, HEIGHT/2+BORDER/2, WIDTH-2*BORDER, HEIGHT/2-3/2*BORDER);
		glutDisplayFunc(constructScene);
		glutKeyboardFunc(keyPressed);
		//glutIdleFunc(&constructScene);
		//InitGL();
		defaultGL();
	glutIdleFunc(idle);
	glutMainLoop();
}

/*
 * MAIN: main loop
 * Init libfreenect, OpenGL (GLUT windows)
 */
int main(int argc, char **argv) {
	//Get Kinect Device
	device = &freenect.createDevice<MyFreenectDevice>(0);
	//Start Kinect Device
	//device->setTiltDegrees(10);
	device->startVideo();
	device->startDepth();
	//handle Kinect Device Data
	device->setLed(LED_GREEN);
	setUpDisplay(device);
	//cout << "Focal Length: " << reference_distance;

	//Stop Kinect Device
	device->stopVideo();
	device->stopDepth();
	device->setLed(LED_OFF);

	//RETURN
	return 1;
}