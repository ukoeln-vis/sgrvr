// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include "async/connection.h"
#include "async/connection_manager.h"

#include <exception>
#include <future>
#include <memory>

#include <GL/glew.h>

#include <Support/CmdLine.h>
#include <Support/CmdLineUtil.h>

#include <visionaray/detail/platform.h>

#include <visionaray/bvh.h>
#include <visionaray/camera.h>
#include <visionaray/cpu_buffer_rt.h>
#include <visionaray/scheduler.h>
#include <visionaray/surface.h>
#include <visionaray/traverse.h>

#include <common/manip/arcball_manipulator.h>
#include <common/manip/pan_manipulator.h>
#include <common/manip/zoom_manipulator.h>

#include <common/timer.h>
#include <common/viewer_glut.h>

#include "serialization.h"

using namespace visionaray;
namespace asio = boost::asio;


using namespace async;

enum MessageType
{
    MT_Unknown,
    MT_Spheres,
    MT_Colors,
};


std::istream& operator>>(std::istream& in, camera& cam)
{
    vec3 eye;
    vec3 center;
    vec3 up;

    in >> eye >> std::ws >> center >> std::ws >> up >> std::ws;
    cam.look_at(eye, center, up);

    return in;
}

std::ostream& operator<<(std::ostream& out, camera const& cam)
{
    out << cam.eye() << '\n';
    out << cam.center() << '\n';
    out << cam.up() << '\n';
    return out;
}


//-------------------------------------------------------------------------------------------------
// Solid sphere, one that a ray can travel through from tnear to tfar
//

struct solid_sphere : basic_sphere<float>
{
};

template <typename S>
struct solid_hit_record : visionaray::hit_record<basic_ray<S>, primitive<unsigned>>
{
    S dist;
};

template <typename S, typename Cond>
void update_if(solid_hit_record<S>& dst, solid_hit_record<S> const& src, Cond const& cond)
{
    dst.hit        |= cond;
    dst.t           = select( cond, src.t, dst.t );
    dst.prim_id     = select( cond, src.prim_id, dst.prim_id );
    dst.geom_id     = select( cond, src.geom_id, dst.geom_id );
    dst.u           = select( cond, src.u, dst.u );
    dst.v           = select( cond, src.v, dst.v );

    dst.dist        = select( cond, src.dist, dst.dist );
}

void split_primitive(aabb& L, aabb& R, float plane, int axis, solid_sphere const& prim)
{
    split_primitive(L, R, plane, axis, static_cast<basic_sphere<float>>(prim));
}


template <typename S>
solid_hit_record<S> intersect(
        basic_ray<S> const& ray,
        solid_sphere const& sphere
        )
{
    typedef basic_ray<S> ray_type;
    typedef vector<3, S> vec_type;

    ray_type r = ray;
    r.ori -= vec_type( sphere.center );

    auto A = dot(r.dir, r.dir);
    auto B = dot(r.dir, r.ori) * S(2.0);
    auto C = dot(r.ori, r.ori) - sphere.radius * sphere.radius;

    // solve Ax**2 + Bx + C
    auto disc = B * B - S(4.0) * A * C;
    auto valid = disc >= S(0.0);

    auto root_disc = select(valid, sqrt(disc), disc);

    auto q = select( B < S(0.0), S(-0.5) * (B - root_disc), S(-0.5) * (B + root_disc) );

    auto tnear = q / A;
    auto tfar = C / q;

    auto mask = tnear > tfar;
    auto tmp = select(mask, tnear, S(0.0));
    tnear = select(mask, tfar, tnear);
    tfar = select(mask, tmp, tfar);

    valid &= tnear > S(0.0);

    solid_hit_record<S> result;
    result.hit = valid;
    result.prim_id = sphere.prim_id;
    result.geom_id = sphere.geom_id;
    result.t = select( valid, tnear, S(-1.0) );
    result.dist = select(
            valid,
            sphere.radius * S(64.0) * ((tfar - tnear) / (sphere.radius + sphere.radius)), // TODO: 128 relates to #voxels
            S(0.0)
            ); // TODO: what if ray inside sphere
    return result;
}


//-------------------------------------------------------------------------------------------------
// struct with state variables
//

struct renderer : viewer_glut
{
    using host_ray_type = basic_ray<simd::float4>;

    using bvh_ref = index_bvh<solid_sphere>::bvh_ref;


    renderer()
        : viewer_glut(512, 512, "Splatrend Client")
        , host_sched(8)
        , bbox({ -1.0f, -1.0f, -0.447236f }, { 1.0f, 1.0f, 0.447236f })
    {
        using namespace support;

        add_cmdline_option( cl::makeOption<std::string&>(
            cl::Parser<>(),
            "hostname",
            cl::ArgRequired,
            cl::Desc("IPv4 Host"),
            cl::init(this->hostname)
            ) );
    }

    camera                                      cam;
    cpu_buffer_rt<PF_RGBA32F, PF_UNSPECIFIED>   host_rt;
    tiled_sched<host_ray_type>                  host_sched;

    index_bvh<solid_sphere>                     host_bvh_back;          // "backbuffer" BVH
    index_bvh<solid_sphere>                     host_bvh_front;         // "frontbuffer" BVH

    unsigned                                    frame_num       = 0;

    aabb                                        bbox;
    std::vector<solid_sphere>                spheres_back;           // "backbuffer" spheres
    std::vector<solid_sphere>                spheres_front;          // "frontbuffer" spheres
    std::vector<vec4>                        colors_back;            // "backbuffer" colors
    std::vector<vec4>                        colors_front;           // "frontbuffer" colors

    std::string                                 hostname        = "localhost";

    std::mutex                                  mtx;
    std::condition_variable                     can_render;
    std::condition_variable                     cond;
    bool                                        can_swap        = true;

protected:

    void on_display();
    void on_key_press(key_event const& event);
    void on_mouse_move(visionaray::mouse_event const& event);
    void on_resize(int w, int h);

};

std::unique_ptr<renderer> rend(nullptr);


//-------------------------------------------------------------------------------------------------
// Draw rend->bbox
//

void draw_bounds()
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixf(rend->cam.get_proj_matrix().data());

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixf(rend->cam.get_view_matrix().data());


    glLineWidth(3);

    auto l = rgb_to_luminance(rend->background_color());

    if (l < 0.5f)
    {
        glColor3f(1.0f, 1.0f, 1.0f);
    }
    else
    {
        glColor3f(0.0f, 0.0f, 0.0f);
    }

    glBegin(GL_LINES);

        glVertex3f(rend->bbox.min.x, rend->bbox.min.y, rend->bbox.min.z);
        glVertex3f(rend->bbox.max.x, rend->bbox.min.y, rend->bbox.min.z);

        glVertex3f(rend->bbox.max.x, rend->bbox.min.y, rend->bbox.min.z);
        glVertex3f(rend->bbox.max.x, rend->bbox.max.y, rend->bbox.min.z);

        glVertex3f(rend->bbox.max.x, rend->bbox.max.y, rend->bbox.min.z);
        glVertex3f(rend->bbox.min.x, rend->bbox.max.y, rend->bbox.min.z);

        glVertex3f(rend->bbox.min.x, rend->bbox.max.y, rend->bbox.min.z);
        glVertex3f(rend->bbox.min.x, rend->bbox.min.y, rend->bbox.min.z);

        glVertex3f(rend->bbox.min.x, rend->bbox.min.y, rend->bbox.max.z);
        glVertex3f(rend->bbox.max.x, rend->bbox.min.y, rend->bbox.max.z);

        glVertex3f(rend->bbox.max.x, rend->bbox.min.y, rend->bbox.max.z);
        glVertex3f(rend->bbox.max.x, rend->bbox.max.y, rend->bbox.max.z);

        glVertex3f(rend->bbox.max.x, rend->bbox.max.y, rend->bbox.max.z);
        glVertex3f(rend->bbox.min.x, rend->bbox.max.y, rend->bbox.max.z);

        glVertex3f(rend->bbox.min.x, rend->bbox.max.y, rend->bbox.max.z);
        glVertex3f(rend->bbox.min.x, rend->bbox.min.y, rend->bbox.max.z);

        glVertex3f(rend->bbox.min.x, rend->bbox.min.y, rend->bbox.min.z);
        glVertex3f(rend->bbox.min.x, rend->bbox.min.y, rend->bbox.max.z);

        glVertex3f(rend->bbox.max.x, rend->bbox.min.y, rend->bbox.min.z);
        glVertex3f(rend->bbox.max.x, rend->bbox.min.y, rend->bbox.max.z);

        glVertex3f(rend->bbox.max.x, rend->bbox.max.y, rend->bbox.min.z);
        glVertex3f(rend->bbox.max.x, rend->bbox.max.y, rend->bbox.max.z);

        glVertex3f(rend->bbox.min.x, rend->bbox.max.y, rend->bbox.min.z);
        glVertex3f(rend->bbox.min.x, rend->bbox.max.y, rend->bbox.max.z);

    glEnd();


    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}


//-------------------------------------------------------------------------------------------------
// BVH traversal costs
//

struct traversal_cost_intersector : basic_intersector<traversal_cost_intersector>
{
    using basic_intersector<traversal_cost_intersector>::operator();

    template <typename R, typename S, typename ...Args>
    auto operator()(R const& ray, basic_aabb<S> const& box, Args&&... args)
        -> decltype( intersect(ray, box, std::forward<Args>(args)...) )
    {
        ++num_boxes;
        return intersect(ray, box, std::forward<Args>(args)...);
    }

    template <typename R>
    auto operator()(R const& ray, solid_sphere const& s)
        -> decltype( intersect(ray, s) )
    {
        ++num_spheres;
        return intersect(ray, s);
    }

    template <typename R>
    auto operator()(R const& ray, renderer::bvh_ref const& b)
        -> decltype( intersect(ray, b) )
    {
        ++num_bvhs;
        return intersect(ray, b);
    }

    unsigned num_boxes = 0;
    unsigned num_spheres  = 0;
    unsigned num_bvhs = 0;
};

//-------------------------------------------------------------------------------------------------
// Display function, implements the AO kernel
//

void renderer::on_display()
{
    // cannot swap data
    {
        std::unique_lock<std::mutex> lck(rend->mtx);
        rend->can_swap = false;
    }


    // some setup

    using R = renderer::host_ray_type;
    using S = R::scalar_type;
    using C = vector<4, S>;
//  using V = vector<3, S>;

    auto mv = (mat4(
                90.0 * 0.007045,0,0,0,
                0,90.0 * 0.007045,0,0,
                0,0,40.2513 * 0.007045,0,
                0,0,-2,1
                ));

    auto pr = (mat4(
                2.41421,0,0,0,
                0,2.41421,0,0,
                0,0,-1.0002,-1,
                0,0,-0.020002,0
                ));

    auto vp = rend->cam.get_viewport();

    auto sparams = make_sched_params<pixel_sampler::jittered_blend_type>(
            rend->cam,
//            mv, pr, vp,
            rend->host_rt
            );


    std::vector<renderer::bvh_ref> bvhs;
    bvhs.clear();
    bvhs.push_back(rend->host_bvh_front.ref());

    auto prims_begin = bvhs.data();
    auto prims_end   = bvhs.data() + bvhs.size();

    timer t;

    int max_sphere_interactions = 0;
    int cnt = 0;

    if (rend->host_bvh_front.primitives().size() == 0)
    {
        goto draw;
    }

    rend->host_sched.frame([&](R ray, sampler<S>& /*samp*/) -> result_record<S>
    {
        result_record<S> result;
        result.color = C(0.0);

        C dst(0.0);

        traversal_cost_intersector intersector;

        auto hit_rec = closest_hit(
                ray,
                prims_begin,
                prims_end,
                intersector
                );

        result.hit = hit_rec.hit;
        result.isect_pos  = hit_rec.isect_pos;

        while ( any(hit_rec.hit && dst.w < S(0.95)) )
        {
            VSNRAY_ALIGN(16) int prim_id[4] = { -1, -1, -1, -1 };
            store( prim_id, hit_rec.prim_id );

            // get_color(prim_id)
            auto c1 = rend->colors_front[prim_id[0]];
            auto c2 = rend->colors_front[prim_id[1]];
            auto c3 = rend->colors_front[prim_id[2]];
            auto c4 = rend->colors_front[prim_id[3]];

            auto src = simd::pack( c1, c2, c3, c4 );

            // weight by traveled distanced through sphere
            src.w = mul(src.w, hit_rec.dist, hit_rec.hit, src.w);

            // premultiplied alpha
            auto premult = src.xyz() * src.w;
            src = C(premult, src.w);

            hit_rec.isect_pos = ray.ori + ray.dir * hit_rec.t;
            dst              += select( hit_rec.hit, src * (1.0f - dst.w), C(0.0) );
            ray.ori           = hit_rec.isect_pos + ray.dir * S(1E-5f);

            hit_rec = closest_hit(
                    ray,
                    prims_begin,
                    prims_end,
                    intersector
                    );
        }

        auto t = intersector.num_spheres;
        max_sphere_interactions += t;
        ++cnt;

      result.color = C(temperature_to_rgb(t / S(5000.0)), S(1.0f));
//        result.color += dst;

        return result;
    }, sparams, ++rend->frame_num);

    std::cerr << t.elapsed() << std::endl;
    std::cerr << cnt << ' ' << (int)(max_sphere_interactions / (float)cnt) << std::endl;

    // display the rendered image

draw:
    auto bgcolor = rend->background_color();
    glClearColor(bgcolor.x, bgcolor.y, bgcolor.z, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    rend->host_rt.display_color_buffer();

    if (0)
    {
        draw_bounds();
    }

    swap_buffers();


    // can swap data
    {
        std::unique_lock<std::mutex> lck(rend->mtx);
        rend->can_swap = true;
        rend->cond.notify_one();
    }
}


void renderer::on_key_press(key_event const& event)
{
    static const std::string camera_filename = "visionaray-camera.txt";

    switch (event.key())
    {
    case 'u':
        {
            std::ofstream file( camera_filename );
            if (file.good())
            {
                std::cout << "Storing camera to file: " << camera_filename << '\n';
                file << rend->cam;
            }
        }
        break;

    case 'v':
        {
            std::ifstream file( camera_filename );
            if (file.good())
            {
                file >> rend->cam;
                rend->frame_num = 0;
                std::cout << "Load camera from file: " << camera_filename << '\n';
            }
        }
        break;

    default:
        break;
    }

    viewer_glut::on_key_press(event);
}


//-------------------------------------------------------------------------------------------------
// mouse handling
//

void renderer::on_mouse_move(visionaray::mouse_event const& event)
{
    if (event.get_buttons() != mouse::NoButton)
    {
        rend->frame_num = 0;
    }

    viewer_glut::on_mouse_move(event);
}


//-------------------------------------------------------------------------------------------------
// resize event
//

void renderer::on_resize(int w, int h)
{
    rend->frame_num = 0;

    rend->cam.set_viewport(0, 0, w, h);
    float aspect = w / static_cast<float>(h);
    rend->cam.perspective(45.0f * constants::degrees_to_radians<float>(), aspect, 0.001f, 1000.0f);
    rend->host_rt.resize(w, h);

    viewer_glut::on_resize(w, h);
}


void build_bvh_back()
{
    std::cout << "Creating BVH...\n";

    timer t;

    rend->host_bvh_back = build<index_bvh<solid_sphere>>(
            rend->spheres_back.data(),
            rend->spheres_back.size(),
            false /* spatial splits */
            );

    std::cout << "Time elapsed: " << t.elapsed() << '\n';
}


//-------------------------------------------------------------------------------------------------
// Get payload over TCP
//

struct client
{
    ConnectionManagerPointer manager;
    ConnectionPointer conn;

    client()
        : manager(makeConnectionManager())
    {
    }

    void connect(std::string const& host = "localhost", unsigned short port = 31050)
    {
        std::cout << "client: connecting...\n";

        manager->connect(host, port, boost::bind(&client::handle_new_connection, this, _1, _2));
    }

    void run()
    {
        manager->run_in_thread();
    }

    void wait()
    {
        manager->wait();
    }

    void handle_message(Connection::Reason reason, MessagePointer message, boost::system::error_code const& e)
    {
        if (e)
        {
            std::cout << "client: error: " << e.message() << "\n";

            manager->stop();
            return;
        }

        if (reason == Connection::Read)
        {
            // Read complete.
            // Got a message from the server. Handle it.

            // Order of messages is MT_Spheres, MT_Colors.

            if (message->type() == MT_Spheres)
            {
                std::cout << "client: got spheres...\n";

                splatrend::deserialize(rend->spheres_back, message->data(), message->data() + message->size());

                rend->frame_num = 0;

                return;
            }

            if (message->type() == MT_Colors)
            {
                std::cout << "client: got colors...\n";

                splatrend::deserialize(rend->colors_back, message->data(), message->data() + message->size());

                // Rebuild the BVH etc...
                // Blocking :-(
                rebuild();

                rend->frame_num = 0;

                return;
            }
        }
        else
        {
            // Write complete.
            // Nothing to do here.
        }
    }

    bool handle_new_connection(ConnectionPointer new_conn, boost::system::error_code const& e)
    {
        if (e)
        {
            std::cout << "client: could not connect to server: " << e.message() << "\n";

            manager->stop();
            return false;
        }

        std::cout << "client: connected.\n";

        // Accept and save this connection
        // and set the message handler of the connection.
        conn = new_conn;
        conn->set_handler(boost::bind(&client::handle_message, this, _1, _2, _3));

        return true;
    }

private:

    void rebuild()
    {
        std::cout << "client: rebuilding...\n";

        build_bvh_back();

        // swap "back" and "front" buffers
        {
            std::unique_lock<std::mutex> lck(rend->mtx);

            rend->cond.wait(lck, [&](){ return rend->can_swap == true; });

            using std::swap;

            swap(rend->spheres_front, rend->spheres_back);
            swap(rend->colors_front, rend->colors_back);
            swap(rend->host_bvh_front, rend->host_bvh_back);

            std::cout << "client: buffers swapped.\n";
        }
    }
};


//-------------------------------------------------------------------------------------------------
// Main function, performs initialization
//

int main(int argc, char** argv)
{
    rend = std::unique_ptr<renderer>(new renderer);

    try
    {
        rend->init(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    glewInit();


    // obtain payload

    std::cout << "Receiving data...\n";

    client cli;

    cli.connect(rend->hostname, 31050);
    cli.run();
////    cli.wait();

    float aspect = rend->width() / static_cast<float>(rend->height());

    rend->cam.perspective(45.0f * constants::degrees_to_radians<float>(), aspect, 0.01f, 100.0f);
    rend->cam.set_viewport(0, 0, 512, 512);
    rend->cam.view_all( rend->bbox );

    rend->add_manipulator( std::make_shared<arcball_manipulator>(rend->cam, mouse::Left) );
    rend->add_manipulator( std::make_shared<pan_manipulator>(rend->cam, mouse::Middle) );
    rend->add_manipulator( std::make_shared<zoom_manipulator>(rend->cam, mouse::Right) );

    glViewport(0, 0, 512, 512);
    rend->host_rt.resize(512, 512);

    rend->event_loop();
}
