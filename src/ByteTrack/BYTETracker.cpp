#include "ByteTrack/BYTETracker.h"

#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>
#include<numeric>
#include<iostream>
#include <opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>

byte_track::BYTETracker::BYTETracker(const int& frame_rate,
                                     const int& track_buffer,
                                     const float& track_thresh,
                                     const float& high_thresh,
                                     const float& match_thresh,
                                     const int distance_mode) :
    track_thresh_(track_thresh),
    high_thresh_(high_thresh),
    match_thresh_(match_thresh),
    max_time_lost_(static_cast<size_t>(frame_rate / 30.0 * track_buffer)),
    frame_id_(0),
    track_id_count_(0),
    distance_mode_(distance_mode),
    cmc(byte_track::GMC(2))
{
}

byte_track::BYTETracker::~BYTETracker()
{
}

std::vector<byte_track::BYTETracker::STrackPtr> byte_track::BYTETracker::update(const std::vector<Object>& objects,cv::Mat* img)
{
    ////////////////// Step 1: Get detections //////////////////
    frame_id_++;

    // Create new STracks using the result of object detection
    std::vector<STrackPtr> det_stracks;
    std::vector<STrackPtr> det_low_stracks;

    for (const auto &object : objects)
    {
        const auto strack = std::make_shared<STrack>(object.rect, object.prob,object.rect_feature,object.label);
        if (object.prob >= track_thresh_)
        {
            det_stracks.push_back(strack);
        }
        else
        {
            det_low_stracks.push_back(strack);
        }
    }

    // Create lists of existing STrack
    std::vector<STrackPtr> active_stracks;
    std::vector<STrackPtr> non_active_stracks;
    std::vector<STrackPtr> strack_pool;

    for (const auto& tracked_strack : tracked_stracks_)
    {
        if (!tracked_strack->isActivated())
        {
            non_active_stracks.push_back(tracked_strack);
        }
        else
        {
            active_stracks.push_back(tracked_strack);
        }
    }

    strack_pool = jointStracks(active_stracks, lost_stracks_);

    // Predict current pose by KF
    //std::cout<<strack_pool.size()<<std::endl;
    for (auto &strack : strack_pool)
    {
        strack->predict();
        //strack->multi_gmc();
        //strack->updateRect();
    }
    //std::cout<<"sjkkds"<<img.size()<<std::endl;
    //Eigen::Matrix<float, 2, 3, Eigen::RowMajor> warp_mat=cmc.applySparseOptFlow(img);
    


    //std::cout<<strack_pool.size()<<std::endl;
    ////////////////// Step 2: First association, with IoU //////////////////
    std::vector<STrackPtr> current_tracked_stracks;
    std::vector<STrackPtr> remain_tracked_stracks;
    std::vector<STrackPtr> remain_det_stracks;
    std::vector<STrackPtr> refind_stracks;

    {
        std::vector<std::vector<int>> matches_idx;
        std::vector<int> unmatch_detection_idx, unmatch_track_idx;
        
        const auto dists = calcIouDistance(strack_pool, det_stracks);
        //const auto dists =calcfeatureDistance(strack_pool, det_stracks);
        //std::cout<<dists.size()<<std::endl;
        //std::cout<<"wong"<<std::endl;
        // std::cout<<dists.size()<<std::endl;
        // if (distance_mode_==0){
        //     dists = calcIouDistance(strack_pool, det_stracks);
        //     //std::cout<<dists[0][0]<<std::endl;   //////////////////////////////////////////
        // }
        // else{
        //     dists = calcfeatureDistance(strack_pool, det_stracks);
        // }
        //std::cout<<dists[0][0]<<std::endl; 
        
        linearAssignment(dists, strack_pool.size(), det_stracks.size(), match_thresh_,
                         matches_idx, unmatch_track_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            const auto track = strack_pool[match_idx[0]];
            const auto det = det_stracks[match_idx[1]];
            if (track->getSTrackState() == STrackState::Tracked)
            {
                track->update(*det, frame_id_);
                current_tracked_stracks.push_back(track);
            }
            else
            {
                track->reActivate(*det, frame_id_);
                refind_stracks.push_back(track);
            }
        }

        for (const auto &unmatch_idx : unmatch_detection_idx)
        {
            remain_det_stracks.push_back(det_stracks[unmatch_idx]);
        }

        for (const auto &unmatch_idx : unmatch_track_idx)
        {
            if (strack_pool[unmatch_idx]->getSTrackState() == STrackState::Tracked)
            {
                remain_tracked_stracks.push_back(strack_pool[unmatch_idx]);
            }
        }
    }

    ////////////////// Step 3: Second association, using low score dets //////////////////
    std::vector<STrackPtr> current_lost_stracks;

    {
        std::vector<std::vector<int>> matches_idx;
        std::vector<int> unmatch_track_idx, unmatch_detection_idx;


       
        //const auto dists = calcfeatureDistance(remain_tracked_stracks, det_low_stracks);
        const auto dists = calcIouDistance(remain_tracked_stracks, det_low_stracks);
        // std::cout<<dists.size()<<std::endl;
        // if (distance_mode_==0){

        //     dists = calcIouDistance(remain_tracked_stracks, det_low_stracks);
        // }
        // else{
        //     dists = calcfeatureDistance(remain_tracked_stracks, det_low_stracks);
        // }



        linearAssignment(dists, remain_tracked_stracks.size(), det_low_stracks.size(), 0.5,
                         matches_idx, unmatch_track_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            const auto track = remain_tracked_stracks[match_idx[0]];
            const auto det = det_low_stracks[match_idx[1]];
            if (track->getSTrackState() == STrackState::Tracked)
            {
                track->update(*det, frame_id_);
                current_tracked_stracks.push_back(track);
            }
            else
            {
                track->reActivate(*det, frame_id_);
                refind_stracks.push_back(track);
            }
        }

        for (const auto &unmatch_track : unmatch_track_idx)
        {
            const auto track = remain_tracked_stracks[unmatch_track];
            if (track->getSTrackState() != STrackState::Lost)
            {
                track->markAsLost();
                current_lost_stracks.push_back(track);
            }
        }
    }

    ////////////////// Step 4: Init new stracks //////////////////
    std::vector<STrackPtr> current_removed_stracks;

    {
        std::vector<int> unmatch_detection_idx;
        std::vector<int> unmatch_unconfirmed_idx;
        std::vector<std::vector<int>> matches_idx;

        // Deal with unconfirmed tracks, usually tracks with only one beginning frame
        //const auto dists = calcfeatureDistance(non_active_stracks, remain_det_stracks);
        const auto dists = calcIouDistance(non_active_stracks, remain_det_stracks);
        // std::cout<<dists.size()<<std::endl;
        // if (distance_mode_==0){
        //     dists = calcIouDistance(non_active_stracks, remain_det_stracks);
        // }
        // else{
        //     dists = calcfeatureDistance(non_active_stracks, remain_det_stracks);   ///////////////////////////////
        // }



        linearAssignment(dists, non_active_stracks.size(), remain_det_stracks.size(), 0.7,
                         matches_idx, unmatch_unconfirmed_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            non_active_stracks[match_idx[0]]->update(*remain_det_stracks[match_idx[1]], frame_id_);
            current_tracked_stracks.push_back(non_active_stracks[match_idx[0]]);
        }

        for (const auto &unmatch_idx : unmatch_unconfirmed_idx)
        {
            const auto track = non_active_stracks[unmatch_idx];
            track->markAsRemoved();
            current_removed_stracks.push_back(track);
        }

        // Add new stracks
        for (const auto &unmatch_idx : unmatch_detection_idx)
        {
            const auto track = remain_det_stracks[unmatch_idx];
            if (track->getScore() < high_thresh_)
            {
                continue;
            }
            track_id_count_++;
            track->activate(frame_id_, track_id_count_);
            current_tracked_stracks.push_back(track);
        }
    }

    ////////////////// Step 5: Update state //////////////////
    for (const auto &lost_strack : lost_stracks_)
    {
        if (frame_id_ - lost_strack->getFrameId() > max_time_lost_)
        {
            lost_strack->markAsRemoved();
            current_removed_stracks.push_back(lost_strack);
        }
    }

    tracked_stracks_ = jointStracks(current_tracked_stracks, refind_stracks);
    lost_stracks_ = subStracks(jointStracks(subStracks(lost_stracks_, tracked_stracks_), current_lost_stracks), removed_stracks_);
    removed_stracks_ = jointStracks(removed_stracks_, current_removed_stracks);

    std::vector<STrackPtr> tracked_stracks_out, lost_stracks_out;
    removeDuplicateStracks(tracked_stracks_, lost_stracks_, tracked_stracks_out, lost_stracks_out);
    tracked_stracks_ = tracked_stracks_out;
    lost_stracks_ = lost_stracks_out;

    std::vector<STrackPtr> output_stracks;
    for (const auto &track : tracked_stracks_)
    {
        if (track->isActivated())
        {
            output_stracks.push_back(track);
        }
    }

    return output_stracks;
}
std::vector<byte_track::BYTETracker::STrackPtr> byte_track::BYTETracker::jointStracks(const std::vector<STrackPtr> &a_tlist,
                                                                                      const std::vector<STrackPtr> &b_tlist) const
{
    std::map<int, int> exists;
    std::vector<STrackPtr> res;
    for (size_t i = 0; i < a_tlist.size(); i++)
    {
        exists.emplace(a_tlist[i]->getTrackId(), 1);
        res.push_back(a_tlist[i]);
    }
    for (size_t i = 0; i < b_tlist.size(); i++)
    {
        const int &tid = b_tlist[i]->getTrackId();
        if (!exists[tid] || exists.count(tid) == 0)
        {
            exists[tid] = 1;
            res.push_back(b_tlist[i]);
        }
    }
    return res;
}

std::vector<byte_track::BYTETracker::STrackPtr> byte_track::BYTETracker::subStracks(const std::vector<STrackPtr> &a_tlist,
                                                                                    const std::vector<STrackPtr> &b_tlist) const
{
    std::map<int, STrackPtr> stracks;
    for (size_t i = 0; i < a_tlist.size(); i++)
    {
        stracks.emplace(a_tlist[i]->getTrackId(), a_tlist[i]);
    }

    for (size_t i = 0; i < b_tlist.size(); i++)
    {
        const int &tid = b_tlist[i]->getTrackId();
        if (stracks.count(tid) != 0)
        {
            stracks.erase(tid);
        }
    }

    std::vector<STrackPtr> res;
    std::map<int, STrackPtr>::iterator it;
    for (it = stracks.begin(); it != stracks.end(); ++it)
    {
        res.push_back(it->second);
    }

    return res;
}

void byte_track::BYTETracker::removeDuplicateStracks(const std::vector<STrackPtr> &a_stracks,
                                                     const std::vector<STrackPtr> &b_stracks,
                                                     std::vector<STrackPtr> &a_res,
                                                     std::vector<STrackPtr> &b_res) const
{
    const auto ious = calcIouDistance(a_stracks, b_stracks);
    //const auto ious =calcfeatureDistance(a_stracks, b_stracks);
    
    std::vector<std::pair<size_t, size_t>> overlapping_combinations;
    for (size_t i = 0; i < ious.size(); i++)
    {
        for (size_t j = 0; j < ious[i].size(); j++)
        {
            if (ious[i][j] < 0.15)
            {
                overlapping_combinations.emplace_back(i, j);
            }
        }
    }

    std::vector<bool> a_overlapping(a_stracks.size(), false), b_overlapping(b_stracks.size(), false);
    for (const auto &[a_idx, b_idx] : overlapping_combinations)
    {
        const int timep = a_stracks[a_idx]->getFrameId() - a_stracks[a_idx]->getStartFrameId();
        const int timeq = b_stracks[b_idx]->getFrameId() - b_stracks[b_idx]->getStartFrameId();
        if (timep > timeq)
        {
            b_overlapping[b_idx] = true;
        }
        else
        {
            a_overlapping[a_idx] = true;
        }
    }

    for (size_t ai = 0; ai < a_stracks.size(); ai++)
    {
        if (!a_overlapping[ai])
        {
            a_res.push_back(a_stracks[ai]);
        }
    }

    for (size_t bi = 0; bi < b_stracks.size(); bi++)
    {
        if (!b_overlapping[bi])
        {
            b_res.push_back(b_stracks[bi]);
        }
    }
}

void byte_track::BYTETracker::linearAssignment(const std::vector<std::vector<float>> &cost_matrix,
                                               const int &cost_matrix_size,
                                               const int &cost_matrix_size_size,
                                               const float &thresh,
                                               std::vector<std::vector<int>> &matches,
                                               std::vector<int> &a_unmatched,
                                               std::vector<int> &b_unmatched) const
{
    if (cost_matrix.size() == 0)
    {
        for (int i = 0; i < cost_matrix_size; i++)
        {
            a_unmatched.push_back(i);
        }
        for (int i = 0; i < cost_matrix_size_size; i++)
        {
            b_unmatched.push_back(i);
        }
        return;
    }

    std::vector<int> rowsol; std::vector<int> colsol;
    execLapjv(cost_matrix, rowsol, colsol, true, thresh);
    for (size_t i = 0; i < rowsol.size(); i++)
    {
        if (rowsol[i] >= 0)
        {
            std::vector<int> match;
            match.push_back(i);
            match.push_back(rowsol[i]);
            matches.push_back(match);
        }
        else
        {
            a_unmatched.push_back(i);
        }
    }

    for (size_t i = 0; i < colsol.size(); i++)
    {
        if (colsol[i] < 0)
        {
            b_unmatched.push_back(i);
        }
    }
}

std::vector<std::vector<float>> byte_track::BYTETracker::calcIous(const std::vector<Rect<float>> &a_rect,
                                                                  const std::vector<Rect<float>> &b_rect) const
{
    std::vector<std::vector<float>> ious;
    if (a_rect.size() * b_rect.size() == 0)
    {
        return ious;
    }

    ious.resize(a_rect.size());
    for (size_t i = 0; i < ious.size(); i++)
    {
        ious[i].resize(b_rect.size());
    }

    for (size_t bi = 0; bi < b_rect.size(); bi++)
    {
        for (size_t ai = 0; ai < a_rect.size(); ai++)
        {
            ious[ai][bi] = b_rect[bi].calcIoU(a_rect[ai]);
        }
    }
    //std::cout<<"djj"<<ious.size()<<std::endl;
    return ious;
}

std::vector<std::vector<float>> byte_track::BYTETracker::calcdistance(const std::vector<std::vector<float>> &a_rect,
                                                                      const std::vector<std::vector<float>> &b_rect) const
{
    std::vector<std::vector<float>> ious;
    if (a_rect.size() * b_rect.size() == 0)
    {
        return ious;
    }

    ious.resize(a_rect.size());
    for (size_t i = 0; i < ious.size(); i++)
    {
        ious[i].resize(b_rect.size());
    }

    for (size_t bi = 0; bi < b_rect.size(); bi++)
    {
        for (size_t ai = 0; ai < a_rect.size(); ai++)
        {
            ious[ai][bi]=std::inner_product(a_rect[ai].begin(),a_rect[ai].end(),b_rect[bi].begin(),0);
        }
    }
    //std::cout<<"djjs"<<ious.size()<<std::endl;
    return ious;
}


std::vector<std::vector<float>> byte_track::BYTETracker::calcIouDistance(const std::vector<STrackPtr> &a_tracks,
                                                                          const std::vector<STrackPtr> &b_tracks) const
{
    std::vector<byte_track::Rect<float>> a_rects, b_rects;
    for (size_t i = 0; i < a_tracks.size(); i++)
    {
        a_rects.push_back(a_tracks[i]->getRect());
    }

    for (size_t i = 0; i < b_tracks.size(); i++)
    {
        b_rects.push_back(b_tracks[i]->getRect());
    }

    const auto ious = calcIous(a_rects, b_rects);

    std::vector<std::vector<float>> cost_matrix;
    for (size_t i = 0; i < ious.size(); i++)
    {
        std::vector<float> iou;
        for (size_t j = 0; j < ious[i].size(); j++)
        {
            iou.push_back(1 - ious[i][j]);
        }
        cost_matrix.push_back(iou);
    }
    //std::cout<<"djjs"<<cost_matrix.size()<<std::endl;

    return cost_matrix;
}

std::vector<std::vector<float>> byte_track::BYTETracker::calcfeatureDistance(const std::vector<STrackPtr> &a_tracks,
                                                                          const std::vector<STrackPtr> &b_tracks) const
{
    std::vector<std::vector<float>> a_rects, b_rects;
    for (size_t i = 0; i < a_tracks.size(); i++)
    {
        a_rects.push_back(a_tracks[i]->getfeature());
    }

    for (size_t i = 0; i < b_tracks.size(); i++)
    {
        b_rects.push_back(b_tracks[i]->getfeature());
    }

    const auto ious = calcdistance(a_rects, b_rects);

    std::vector<std::vector<float>> cost_matrix;
    for (size_t i = 0; i < ious.size(); i++)
    {
        std::vector<float> iou;
        for (size_t j = 0; j < ious[i].size(); j++)
        {
            iou.push_back(1 - ious[i][j]);
        }
        cost_matrix.push_back(iou);
    }
    //std::cout<<"akkas"<<std::endl;
    return cost_matrix;
}                                                                          


double byte_track::BYTETracker::execLapjv(const std::vector<std::vector<float>> &cost,
                                          std::vector<int> &rowsol,
                                          std::vector<int> &colsol,
                                          bool extend_cost,
                                          float cost_limit,
                                          bool return_cost) const
{
    std::vector<std::vector<float> > cost_c;
    cost_c.assign(cost.begin(), cost.end());

    std::vector<std::vector<float> > cost_c_extended;

    int n_rows = cost.size();
    int n_cols = cost[0].size();
    rowsol.resize(n_rows);
    colsol.resize(n_cols);

    int n = 0;
    if (n_rows == n_cols)
    {
        n = n_rows;
    }
    else
    {
        if (!extend_cost)
        {
            throw std::runtime_error("The `extend_cost` variable should set True");
        }
    }

    if (extend_cost || cost_limit < std::numeric_limits<float>::max())
    {
        n = n_rows + n_cols;
        cost_c_extended.resize(n);
        for (size_t i = 0; i < cost_c_extended.size(); i++)
            cost_c_extended[i].resize(n);

        if (cost_limit < std::numeric_limits<float>::max())
        {
            for (size_t i = 0; i < cost_c_extended.size(); i++)
            {
                for (size_t j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_limit / 2.0;
                }
            }
        }
        else
        {
            float cost_max = -1;
            for (size_t i = 0; i < cost_c.size(); i++)
            {
                for (size_t j = 0; j < cost_c[i].size(); j++)
                {
                    if (cost_c[i][j] > cost_max)
                        cost_max = cost_c[i][j];
                }
            }
            for (size_t i = 0; i < cost_c_extended.size(); i++)
            {
                for (size_t j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_max + 1;
                }
            }
        }

        for (size_t i = n_rows; i < cost_c_extended.size(); i++)
        {
            for (size_t j = n_cols; j < cost_c_extended[i].size(); j++)
            {
                cost_c_extended[i][j] = 0;
            }
        }
        for (int i = 0; i < n_rows; i++)
        {
            for (int j = 0; j < n_cols; j++)
            {
                cost_c_extended[i][j] = cost_c[i][j];
            }
        }

        cost_c.clear();
        cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
    }

    double **cost_ptr;
    cost_ptr = new double *[sizeof(double *) * n];
    for (int i = 0; i < n; i++)
        cost_ptr[i] = new double[sizeof(double) * n];

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cost_ptr[i][j] = cost_c[i][j];
        }
    }

    int* x_c = new int[sizeof(int) * n];
    int *y_c = new int[sizeof(int) * n];

    int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
    if (ret != 0)
    {
        throw std::runtime_error("The result of lapjv_internal() is invalid.");
    }

    double opt = 0.0;

    if (n != n_rows)
    {
        for (int i = 0; i < n; i++)
        {
            if (x_c[i] >= n_cols)
                x_c[i] = -1;
            if (y_c[i] >= n_rows)
                y_c[i] = -1;
        }
        for (int i = 0; i < n_rows; i++)
        {
            rowsol[i] = x_c[i];
        }
        for (int i = 0; i < n_cols; i++)
        {
            colsol[i] = y_c[i];
        }

        if (return_cost)
        {
            for (size_t i = 0; i < rowsol.size(); i++)
            {
                if (rowsol[i] != -1)
                {
                    opt += cost_ptr[i][rowsol[i]];
                }
            }
        }
    }
    else if (return_cost)
    {
        for (size_t i = 0; i < rowsol.size(); i++)
        {
            opt += cost_ptr[i][rowsol[i]];
        }
    }

    for (int i = 0; i < n; i++)
    {
        delete[]cost_ptr[i];
    }
    delete[]cost_ptr;
    delete[]x_c;
    delete[]y_c;

    return opt;
}


void byte_track::BYTETracker::clear_all_tracks()
{
    std::vector <STrackPtr>().swap(tracked_stracks_);
    std::vector <STrackPtr>().swap(lost_stracks_);
    std::vector <STrackPtr>().swap(removed_stracks_);
    frame_id_=0;
    track_id_count_=0;
    /////gmc also

}


byte_track::GMC::GMC(int downscale) :
    downscale_(downscale),
    initializedFirstFrame(0)
{
}




Eigen::Matrix<float, 2, 3, Eigen::RowMajor> byte_track::GMC::applySparseOptFlow(cv::Mat img)
{
       
        
        cv::Size whs= img.size();
         cv::Mat img_G;
         cv::cvtColor(img,img_G,cv::COLOR_RGB2GRAY);
         Eigen::Matrix<float, 2, 3, Eigen::RowMajor> H = Eigen::MatrixXf::Identity(2, 3);


        

        // Downscale image
        if (downscale_ > 1.0)
           cv::resize(img_G, img_G,cv::Size(int(whs.width /downscale_), int(whs.height /downscale_)));

        // find the keypoints
        std::vector<cv::Point2f> keypoints;
        cv::goodFeaturesToTrack(img_G, 
                            keypoints,
                            1000, 
                            0.01, 
                            1);
                            // mask=None,
                            // blockSize=3,
                            // useHarrisDetector=False,
                            // k=0.04);

    // Handle first frame
        if (!initializedFirstFrame)
        {
            //Initialize data
            prevFrame = img_G.clone();
            //prevKeyPoints = keypoints.clone();
            prevKeyPoints.resize(keypoints.size());
            std::copy(keypoints.begin(),keypoints.end(),prevKeyPoints.begin());

            // Initialization done
            initializedFirstFrame = true;

            return H;
        }
        //find correspondences
        std::vector<uchar> status;
        std::vector<float> err;
        std::vector<cv::Point2f> matchedkeypoints;
        cv::calcOpticalFlowPyrLK(prevFrame,img_G,prevKeyPoints,matchedkeypoints,status,err);

        // leave good correspondences only
        std::vector<cv::Point2f> prevPoints;
        std::vector<cv::Point2f> currPoints;

        for (int i=0;i<prevKeyPoints.size();i++)
            if (status[i]==1)
            {
                prevPoints.push_back(prevKeyPoints[i]);
                currPoints.push_back(matchedkeypoints[i]);
            }

        //prevPoints = np.array(prevPoints)
        //currPoints = np.array(currPoints)

        //Find rigid matrix
        cv::Mat ps;
        if ((prevPoints.size() > 4) and (prevPoints.size() == prevPoints.size()))
        {
            std::vector<uchar> inliers(prevPoints.size(), 0);
             ps = cv::estimateAffinePartial2D(prevPoints, currPoints,inliers,cv::RANSAC);
        }
        cv::cv2eigen(ps,H);
        //std::cout<<"dk"<<ps.size()<<std::endl;
         
            // Handle downscale
            if (downscale_ > 1.0)
            {
                H(0, 2) *= downscale_;
                H(1, 2) *= downscale_;
            }
            else
                std::cout<<"Warning: not enough matching points"<<std::endl;

        //Store to next iteration
        prevFrame = img_G.clone();
        prevKeyPoints.resize(keypoints.size());
        std::copy(keypoints.begin(),keypoints.end(),prevKeyPoints.begin());

    return H;


}









// class GMC:
//     def __init__(self, method='sparseOptFlow', downscale=2, verbose=None):
//         super(GMC, self).__init__()

//         self.method = method
//         self.downscale = max(1, int(downscale))
        //     self.feature_params = dict(maxCorners=1000, qualityLevel=0.01, minDistance=1, blockSize=3,
        //                                useHarrisDetector=False, k=0.04)
        //     self.prevFrame = None
        // self.prevKeyPoints = None
        // self.prevDescriptors = None

        // self.initializedFirstFrame = False

//     def applySparseOptFlow(self, raw_frame, detections=None):

//         t0 = time.time()

//         # Initialize
//         height, width, _ = raw_frame.shape
//         frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)
//         H = np.eye(2, 3)

//         # Downscale image
//         if self.downscale > 1.0:
//             # frame = cv2.GaussianBlur(frame, (3, 3), 1.5)
//             frame = cv2.resize(frame, (width // self.downscale, height // self.downscale))

//         # find the keypoints
//         keypoints = cv2.goodFeaturesToTrack(frame, mask=None, **self.feature_params)

//         # Handle first frame
//         if not self.initializedFirstFrame:
//             # Initialize data
//             self.prevFrame = frame.copy()
//             self.prevKeyPoints = copy.copy(keypoints)

//             # Initialization done
//             self.initializedFirstFrame = True

//             return H

//         # find correspondences
//         matchedKeypoints, status, err = cv2.calcOpticalFlowPyrLK(self.prevFrame, frame, self.prevKeyPoints, None)

//         # leave good correspondences only
//         prevPoints = []
//         currPoints = []

//         for i in range(len(status)):
//             if status[i]:
//                 prevPoints.append(self.prevKeyPoints[i])
//                 currPoints.append(matchedKeypoints[i])

//         prevPoints = np.array(prevPoints)
//         currPoints = np.array(currPoints)

//         # Find rigid matrix
//         if (np.size(prevPoints, 0) > 4) and (np.size(prevPoints, 0) == np.size(prevPoints, 0)):
//             H, inliesrs = cv2.estimateAffinePartial2D(prevPoints, currPoints, cv2.RANSAC)

//             # Handle downscale
//             if self.downscale > 1.0:
//                 H[0, 2] *= self.downscale
//                 H[1, 2] *= self.downscale
//         else:
//             print('Warning: not enough matching points')

//         # Store to next iteration
//         self.prevFrame = frame.copy()
//         self.prevKeyPoints = copy.copy(keypoints)

//         t1 = time.time()

//         # gmc_line = str(1000 * (t1 - t0)) + "\t" + str(H[0, 0]) + "\t" + str(H[0, 1]) + "\t" + str(
//         #     H[0, 2]) + "\t" + str(H[1, 0]) + "\t" + str(H[1, 1]) + "\t" + str(H[1, 2]) + "\n"
//         # self.gmc_file.write(gmc_line)

//         return H

    