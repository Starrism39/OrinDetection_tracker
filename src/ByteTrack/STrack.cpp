#include "ByteTrack/STrack.h"

#include <cstddef>
#include<vector>
#include<eigen3/unsupported/Eigen/KroneckerProduct>
byte_track::STrack::STrack(const Rect<float>& rect, const float& score,const std::vector<float>& rect_feature,const int& label) :
    kalman_filter_(),
    mean_(),
    covariance_(),
    rect_(rect),
    rect_feature_(rect_feature),
    state_(STrackState::New),
    is_activated_(false),
    score_(score),
    label_(label),
    track_id_(0),
    frame_id_(0),
    start_frame_id_(0),
    tracklet_len_(0)
{
}

byte_track::STrack::~STrack()
{
}

const byte_track::Rect<float>& byte_track::STrack::getRect() const
{
    return rect_;
}

const std::vector<float>& byte_track::STrack::getfeature() const
{
    return rect_feature_;
}

const byte_track::STrackState& byte_track::STrack::getSTrackState() const
{
    return state_;
}

const bool& byte_track::STrack::isActivated() const
{
    return is_activated_;
}
const float& byte_track::STrack::getScore() const
{
    return score_;
}
const int& byte_track::STrack::getlabel() const
{
    return label_;
}

const size_t& byte_track::STrack::getTrackId() const
{
    return track_id_;
}

const size_t& byte_track::STrack::getFrameId() const
{
    return frame_id_;
}

const size_t& byte_track::STrack::getStartFrameId() const
{
    return start_frame_id_;
}

const size_t& byte_track::STrack::getTrackletLength() const
{
    return tracklet_len_;
}

void byte_track::STrack::activate(const size_t& frame_id, const size_t& track_id)
{
    kalman_filter_.initiate(mean_, covariance_, rect_.getXyah());

    updateRect();

    state_ = STrackState::Tracked;
    if (frame_id == 1)
    {
        is_activated_ = true;
    }
    track_id_ = track_id;
    frame_id_ = frame_id;
    start_frame_id_ = frame_id;
    tracklet_len_ = 0;
}

void byte_track::STrack::reActivate(const STrack &new_track, const size_t &frame_id, const int &new_track_id)
{
    kalman_filter_.update(mean_, covariance_, new_track.getRect().getXyah());

    updateRect();

    state_ = STrackState::Tracked;
    is_activated_ = true;
    score_ = new_track.getScore();
    if (0 <= new_track_id)
    {
        track_id_ = new_track_id;
    }
    frame_id_ = frame_id;
    tracklet_len_ = 0;
}

void byte_track::STrack::predict()
{
    if (state_ != STrackState::Tracked)
    {
        mean_[7] = 0;
    }
    kalman_filter_.predict(mean_, covariance_);
}

void byte_track::STrack::update(const STrack &new_track, const size_t &frame_id)
{
    kalman_filter_.update(mean_, covariance_, new_track.getRect().getXyah());

    updateRect();

    state_ = STrackState::Tracked;
    is_activated_ = true;
    score_ = new_track.getScore();
    frame_id_ = frame_id;
    tracklet_len_++;
}

void byte_track::STrack::markAsLost()
{
    state_ = STrackState::Lost;
}

void byte_track::STrack::markAsRemoved()
{
    state_ = STrackState::Removed;
}

void byte_track::STrack::updateRect()
{
    rect_.width() = mean_[2] * mean_[3];
    rect_.height() = mean_[3];
    rect_.x() = mean_[0] - rect_.width() / 2;
    rect_.y() = mean_[1] - rect_.height() / 2;
}

// void byte_track::STrack::multi_gmc(const STrack &new_track,Eigen::Matrix<float, 2, 3, Eigen::RowMajor> H)
// {
//     Eigen::Matrix<float, 2, 2, Eigen::RowMajor> R = H.block(0,0,2,2);
//     Eigen::Matrix<float, 4, 4, Eigen::RowMajor> I=Eigen::MatrixXf::Identify(4,4);
//     Eigen::Matrix<float, 8, 8, Eigen::RowMajor> R8x8=Eigen::kroneckerProduct(I,R);

//     //Eigen::Matrix<float, 2, 2, Eigen::RowMajor> R8x8 = np.kron(np.eye(4, dtype=float), R)
//     Eigen::Matrix<float, 2, 1, Eigen::RowMajor> t = H.block(0,2,2,1);

//     //Eigen::Matrix<float, 1, 8, Eigen::RowMajor> mean=
//     mean_(0,2)=mean_(0,2)*mean_(0,3);
//     mean_(0,6)=mean_(0,6)*mean_(0,7);
//     //mean_ = R8x8*mean_;
//     mean_(0,2)=mean_(0,2)/mean_(0,3);
//     mean_(0,6)=mean_(0,6)/mean_(0,7);

//     mean_.block(0,0,1,2) += t.transpose();
//     covariance_  = R8x8*covariance_*(R8x8.transpose());

//     // stracks.mean_ = mean
//     // stracks.covariance_ = cov
// }

// def multi_gmc(stracks, H=np.eye(2, 3)):
//         if len(stracks) > 0:
//             multi_mean = np.asarray([st.mean.copy() for st in stracks])
//             multi_covariance = np.asarray([st.covariance for st in stracks])

//             R = H[:2, :2]
//             R8x8 = np.kron(np.eye(4, dtype=float), R)
//             t = H[:2, 2]

//             for i, (mean, cov) in enumerate(zip(multi_mean, multi_covariance)):
//                 mean = R8x8.dot(mean)
//                 mean[:2] += t
//                 cov = R8x8.dot(cov).dot(R8x8.transpose())

//                 stracks[i].mean = mean
//                 stracks[i].covariance = cov