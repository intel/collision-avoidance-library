#include "GazeboRealSenseCamera.hh"

// TODO: The camera configuration should be aggregated on the gazebo .sdf model
// file or retrieved from the librealsense. This feature is not implemented
// yet, so those values are stored here until then.
#define GZ_RS_STREAM_DEPTH_TOPIC "~/gzsitl_quadcopter_rs/rs/stream/depth"
#define DEPTH_CAM_WIDTH 640
#define DEPTH_CAM_HEIGHT 480
#define DEPTH_CAM_FOV M_PI / 3.0
#define DEPTH_CAM_SCALE 0.001

GazeboRealSenseCamera::GazeboRealSenseCamera()
{
    // Start communication with Gazebo
    std::cout << "[GazeboRealSenseCamera] Waiting for Gazebo..." << std::endl;
    this->gazebo_context = GazeboContext::instance();

    // Create our node for communication
    this->gznode.reset(gazebo_context->node());
    this->gznode->Init();

    std::cout << "[GazeboRealSenseCamera] Gazebo Initialized" << std::endl;

    // TODO: Retrieve camera data straight from topic or camera plugin
    this->width = DEPTH_CAM_WIDTH;
    this->height = DEPTH_CAM_HEIGHT;
    this->fov = DEPTH_CAM_FOV;
    this->scale = DEPTH_CAM_SCALE;

    // TODO: Find RealSense camera topic and parameters automatically
    this->rs_depth_sub = this->gznode->Subscribe(
        GZ_RS_STREAM_DEPTH_TOPIC, &GazeboRealSenseCamera::on_stream_depth_recvd,
        this);
}

GazeboRealSenseCamera::~GazeboRealSenseCamera()
{
}

std::vector<uint16_t> &GazeboRealSenseCamera::get_depth_buffer()
{
    std::lock_guard<std::mutex> locker(depth_buffer_mtx);
    return depth_buffer[this->current_buffer ^= 1];
}

void GazeboRealSenseCamera::on_stream_depth_recvd(ConstImageStampedPtr &_msg)
{
    if (!this->camera_exists) {
        this->camera_exists = true;
        std::cout << "[GazeboRealSenseCamera] Real Sense Initialized" << std::endl;
    }

    std::lock_guard<std::mutex> locker(depth_buffer_mtx);

    uint16_t *data = (uint16_t *) _msg->image().data().c_str();
    uint buffer_size = _msg->image().width() * _msg->image().height();
    depth_buffer[this->current_buffer ^ 1] = std::vector<uint16_t>(data, data + buffer_size);
}

