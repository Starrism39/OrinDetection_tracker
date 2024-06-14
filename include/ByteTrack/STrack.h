#pragma once

#include "ByteTrack/Rect.h"
#include "ByteTrack/KalmanFilter.h"

#include <cstddef>
#include <vector>
namespace byte_track
{
enum class STrackState {
    New = 0,
    Tracked = 1,
    Lost = 2,
    Removed = 3,
};

class STrack
{
public:
    STrack(const Rect<float>& rect, const float& score,const std::vector<float>& rect_feature,const int& label);
    ~STrack();

    const Rect<float>& getRect() const;
    const STrackState& getSTrackState() const;
    const std::vector<float>& getfeature() const;

    const bool& isActivated() const;
    const float& getScore() const;
    const int& getlabel() const;
    const size_t& getTrackId() const;
    const size_t& getFrameId() const;
    const size_t& getStartFrameId() const;
    const size_t& getTrackletLength() const;

    void activate(const size_t& frame_id, const size_t& track_id);
    void reActivate(const STrack &new_track, const size_t &frame_id, const int &new_track_id = -1);

    void predict();
    void update(const STrack &new_track, const size_t &frame_id);

    void markAsLost();
    void markAsRemoved();
    void updateRect();
    void multi_gmc(const STrack &new_track,Eigen::Matrix<float, 2, 3, Eigen::RowMajor> H);

private:
    KalmanFilter kalman_filter_;
    KalmanFilter::StateMean mean_;
    KalmanFilter::StateCov covariance_;

    Rect<float> rect_;
    STrackState state_;
    std::vector<float> rect_feature_;

    bool is_activated_;
    float score_;
    int label_;
    size_t track_id_;
    size_t frame_id_;
    size_t start_frame_id_;
    size_t tracklet_len_;

    //void updateRect();
};
}