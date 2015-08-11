// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include "async/connection.h"
#include "async/connection_manager.h"

#include <cassert>
#include <exception>
#include <iostream>
#include <memory>
#include <ostream>
#include <random>
#include <thread>
#include <utility>

#include <GL/glew.h>

#undef MATH_NAMESPACE
#include <virvo/vvfileio.h>
#undef MATH_NAMESPACE
#define MATH_NAMESPACE visionaray

#include <visionaray/math/math.h>
#include <visionaray/texture/texture.h>

#include "serialization.h"

using namespace visionaray;
namespace asio = boost::asio;
using boost::asio::ip::tcp;
using sphere = basic_sphere<float>;
using volume_ref            = texture_ref<uint8_t, ElementType, 3>;
using transfer_function_ref = texture_ref<vec4, ElementType, 1>;


using namespace async;

enum MessageType
{
    MT_Unknown,
    MT_Spheres,
    MT_Colors,
};


namespace visionaray
{

//-------------------------------------------------------------------------------------------------
// Tags
//

struct voxel_space_tag {};
struct object_space_tag {};


//-------------------------------------------------------------------------------------------------
// Simple brick list
//

struct bricks
{
    vec3i count = vec3i(0, 0, 0);           // bricks per dimension
    std::vector<aabbi> voxel_space_bricks;  // coordinates range from [ 0..size)
    std::vector<aabb> object_space_bricks;  // coordinates range from [-1..1]

    aabbi get_brick(voxel_space_tag, int i, int j, int k) const
    {
        return voxel_space_bricks[index(i, j, k)];
    }

    aabb  get_brick(object_space_tag, int i, int j, int k) const
    {
        return object_space_bricks[index(i, j, k)];
    }

    int index(int i, int j, int k) const
    {
        return k * count.y * count.x + j * count.x + i;
    }
};

} // visionaray


//-------------------------------------------------------------------------------------------------
// Make bricks from volume
//

bricks make_bricks(volume_ref const& volume, vec3i const& count)
{
    bricks bs;
    bs.count = count;

    vec3i size = vec3i(volume.width(), volume.height(), volume.depth()) / count;
    vec3 osize = ( vec3(size) / vec3(volume.width(), volume.height(), volume.depth()) ) * 2.0f;

    for (int z = 0; z < count.z; ++z)
    {
        for (int y = 0; y < count.y; ++y)
        {
            for (int x = 0; x < count.x; ++x)
            {
                vec3i min( x * size.x, y * size.y, z * size.z );
                vec3i max( min.x + size.x, min.y + size.y, min.z + size.z );
                bs.voxel_space_bricks.emplace_back(min, max);

                float ox = ( x / static_cast<float>(volume.width()) ) * 2.0f - 1.0f;
                float oy = ( y / static_cast<float>(volume.height())) * 2.0f - 1.0f;
                float oz = ( z / static_cast<float>(volume.depth()) ) * 2.0f - 1.0f;

                bs.object_space_bricks.emplace_back(
                        vec3( ox, oy, oz ),
                        vec3( ox + osize.x, oy + osize.y, oz + osize.z )
                        );
            }
        }
    }

    return bs;
}

template <class Func>
static void for_each_voxel(vec3i lower, vec3i upper, Func func)
{
    vec3i p;

    for (p.z = lower.z; p.z < upper.z; ++p.z)
    {
        for (p.y = lower.y; p.y < upper.y; ++p.y)
        {
            for (p.x = lower.x; p.x < upper.x; ++p.x)
            {
                func(p);
            }
        }
    }
}

template <class Func>
static void for_each_brick(vec3i lower, vec3i upper, vec3i size, Func func)
{
    vec3i p;
    vec3i s;

    for (p.z = lower.z; p.z < upper.z; p.z += size.z)
    {
        s.z = min(size.z, upper.z - p.z);

        for (p.y = lower.y; p.y < upper.y; p.y += size.y)
        {
            s.y = min(size.y, upper.y - p.y);

            for (p.x = lower.x; p.x < upper.x; p.x += size.x)
            {
                s.x = min(size.x, upper.x - p.x);

                func(p, s);
            }
        }
    }
}

template <class Func>
static void for_each_brick_aligned(vec3i lower, vec3i upper, vec3i size, Func func)
{
    lower = size * (lower / size);

    return for_each_brick_aligned(lower, upper, size, func);
}


//-------------------------------------------------------------------------------------------------
// Estimate radius based on #samples and the extent of the sampled box
//

float get_radius(vec3 const& extent, int num_samples)
{
    auto bvolume = hmul(extent);

    return std::cbrt(
            (3.0f / 4.0f)
          * (bvolume / (num_samples * visionaray::constants::pi<float>()))
            );
}

float get_radius(aabb const& bounds, int num_samples)
{
    return get_radius(bounds.size(), num_samples);
}


//-------------------------------------------------------------------------------------------------
// Naive sampling
//

void sample_volume(
        volume_ref const& volume,
        transfer_function_ref const& tf,
        std::vector<sphere>& spheres,        // vectors will be appended to!
        std::vector<vec4>& colors,           // vectors will be appended to!
        int num_samples
        )
{
    std::random_device rd;
    std::default_random_engine e1(rd());
    std::uniform_real_distribution<float> pos_dist(0.0f, 1.0f);

    auto vol_size = vec3(volume.size());
    vol_size /= 0.5f * max_element(vol_size);

    for (int i = 0; i < num_samples; ++i)
    {
        auto sample = vec3(pos_dist(e1), pos_dist(e1), pos_dist(e1));

        sphere s;

        s.center = vol_size * sample - vol_size / 2.0f;
        s.center.y = -s.center.y;
        s.center.z = -s.center.z;


        // sample volume file
        auto raw = tex3D(volume, sample);

        // early out?
//      if (raw == 0) continue;


        s.radius = get_radius(vol_size, num_samples);
        s.prim_id = i;
        spheres.push_back(s);
        colors.push_back(tex1D(tf, raw / 255.0f));
    }
}


//-------------------------------------------------------------------------------------------------
// Stratified sampling
//

#if 0
void sample_volume_stratified(
        volume_ref const& volume,
        transfer_function_ref const& tf,
        std::vector<sphere>& spheres,        // vectors will be appended to!
        std::vector<vec4>& colors,           // vectors will be appended to!
        int num_samples
        )
{
    std::random_device rd;
    std::default_random_engine e1(rd());
    std::uniform_real_distribution<float> pos_dist(0.0f, 1.0f);


    vec3i strata(16, 16, 16);

    auto num_strata = hmul(strata);
    auto samples_per_stratum = num_samples / num_strata;

    // adjust num_samples so that samples are equally distributed among strata
    num_samples = samples_per_stratum * num_strata;

    auto vol_size = vec3(volume.size());
    vol_size /= 0.5f * max_element(vol_size);

    auto bs = make_bricks(volume, strata);

    int i = 0;
    for (int z = 0; z < bs.count.z; ++z)
    {
        for (int y = 0; y < bs.count.y; ++y)
        {
            for (int x = 0; x < bs.count.x; ++x)
            {
                auto vox = bs.get_brick(voxel_space_tag(), x, y, z);
                auto obj = bs.get_brick(object_space_tag(), x, y, z);

                for (int samp = 0; samp < samples_per_stratum; ++samp)
                {
                    sphere s;

                    vec3 rnd( pos_dist(e1), pos_dist(e1), pos_dist(e1) );

                    // map to [0..1] texel coordinates
                    auto posf = ( rnd * vec3(vox.size()) + vec3(x, y, z) * vec3(vox.size()) )
                              / vec3(volume.width() - 1, volume.height() - 1, volume.depth() - 1);


                    // sample volume
                    auto raw = tex3D(volume, posf);


                    // early out?
//                  if (raw < 10) continue;


                    // map to [-1..1] object space coordinates
                    s.center = posf * vol_size - vol_size / 2.0f;
                    s.radius = get_radius(obj.size(), samples_per_stratum);
                    s.prim_id = i++;
                    spheres.push_back(s);
                    colors.push_back(tex1D(tf, raw / 255.0f));
                }
            }
        }
    }

//    assert( i == num_samples );
}
#else
void sample_volume_stratified(
        volume_ref const& volume,
        transfer_function_ref const& tf,
        std::vector<sphere>& spheres,        // vectors will be appended to!
        std::vector<vec4>& colors,           // vectors will be appended to!
        int num_samples
        )
{
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<float> pos_dist(0.0f, 1.0f);

    vec3i vsize = vec3i(volume.size());

    // How many strata per dimension?
    vec3i strata(16, 16, 16);

    float svol = (float)hmul(strata);

    // Compute the size per stratum
#if 1
    vec3i ssize = (vsize + strata - 1) / strata;
#else
    vec3i ssize = vsize / strata;
#endif

    float bvol = (float)hmul(ssize);

    // Average number of samples per stratum
    float samples_per_stratum = num_samples / svol;

    vec3f vol_size = vec3(vsize);

    vol_size /= 0.5f * max_element(vol_size);

    int i = 0;

    // Sample each stratum
    for_each_brick({0,0,0}, vsize, ssize, [&](vec3i bp, vec3i bs)
    {
        const int samples = (int)( samples_per_stratum * (hmul(bs) / bvol) + 0.5f );

        for (int k = 0; k < samples; ++k)
        {
            vec3f sample = vec3f(pos_dist(rng), pos_dist(rng), pos_dist(rng));

            // Compute the position of the sample inside the volume
            vec3f sample_pos = vec3f(bp) + sample * vec3f(bs);

            // Map to texture coordinates in [0...1]
            vec3f p = sample_pos / vec3f(vsize - 1);


            // Sample the volume
            auto raw = tex3D(volume, p);

            // early out?
//          if (raw == 0) continue;

            sphere s;

            s.center = (p - 0.5f) * vol_size;
            s.center.y = -s.center.y;
            s.center.z = -s.center.z;
            s.radius = get_radius(2.0f * vec3f(bs) / vec3f(vsize), samples);
            s.prim_id = i++;

            spheres.push_back(s);

            colors.push_back(tex1D(tf, raw / 255.0f));
        }
    });

    std::cout << "generated " << spheres.size() << " spheres.\n";
}
#endif

//-------------------------------------------------------------------------------------------------
// Average values associated with the bricks, and a factory function
//

struct brick_data
{
    brick_data() = default;
    brick_data(float avgd)
        : avg_density(avgd)
    {
    }

    float avg_density;
};

std::vector<brick_data> make_brick_data(bricks const& bs, volume_ref const& volume)
{
    std::vector<brick_data> result;

    for (auto b : bs.voxel_space_bricks)
    {
        auto maxx = min(b.max.x, (int)volume.width());
        auto maxy = min(b.max.y, (int)volume.height());
        auto maxz = min(b.max.z, (int)volume.depth());

        float avg_density = 0;

        for (int z = b.min.z; z < maxz; ++z)
        {
            for (int y = b.min.y; y < maxy; ++y)
            {
                for (int x = b.min.x; x < maxx; ++x)
                {
                    avg_density += volume(x, y, z);// / 255.0f;
                }
            }
        }

        auto bounds = aabbi(b.min, vec3i(maxx, maxy, maxz));

        avg_density /= visionaray::volume(bounds);

        result.emplace_back(avg_density);
    }

    return result;
}


//-------------------------------------------------------------------------------------------------
// A binary XXX-partitioning tree (maybe kd-tree, maybe BVH)
//

struct tree_node
{
    aabbi                       bounds;
    std::shared_ptr<tree_node>  child_left          = nullptr;
    std::shared_ptr<tree_node>  child_right         = nullptr;

    vec3i                       first_brick_index;
    vec3i                       num_bricks;
};


//-------------------------------------------------------------------------------------------------
// Calc stats based on bricks contained in bounds
//

struct stats
{
    float expected_value;
    float variance;
    float std_deviation;
    float rel_deviation;
    vec3i center_of_mass;
};

stats statistics(
        aabbi const& bounds,
        bricks const& bs,
        std::vector<brick_data> const& bd
        )
{
    stats result;

    result.variance = 0;

    float total_density = 0.0f;
    int contained = 0;


    // calc center of mass, total and avg density

    vec3 com(0.0);

    for (size_t i = 0; i < bs.voxel_space_bricks.size(); ++i)
    {
        auto b = bs.voxel_space_bricks[i];
        if (!bounds.contains(b))
        {
            continue;
        }

        com += vec3(b.min) * bd[i].avg_density;
        total_density += bd[i].avg_density;

        ++contained;
    }

    assert( contained > 0 );

    result.expected_value = total_density / contained;
    result.center_of_mass = vec3i(com / total_density);


    // calc variance and count outliers

    for (size_t i = 0; i < bs.voxel_space_bricks.size(); ++i)
    {
        auto b = bs.voxel_space_bricks[i];
        if (!bounds.contains(b))
        {
            continue;
        }

        auto tmp = bd[i].avg_density - result.expected_value;
        result.variance += tmp * tmp;
    }

    result.variance /= contained;
    result.std_deviation = sqrt(result.variance);
    result.rel_deviation = (result.variance == 0 || result.expected_value == 0)
            ? 0
            : result.std_deviation / result.expected_value;

    return result;
}


//-------------------------------------------------------------------------------------------------
// Recursively construct the tree
//

void split(tree_node* node, bricks const& bs, std::vector<brick_data> const& bd)
{
    // use binning to find split plane

    float threshold         = 0.05f;
    int best_pos            = 0;
    int best_axis           = -1;
    auto brick_size         = bs.voxel_space_bricks[0].size();

    auto s = statistics(node->bounds, bs, bd);

    best_axis = (int)max_index(node->bounds.size());

    if (s.rel_deviation > threshold && node->num_bricks[best_axis] > 4)
    {
        // best pos for median split
        best_pos = s.center_of_mass[best_axis];

        if (best_pos == node->bounds.min[best_axis])
        {
            ++best_pos;
        }

        node->child_left  = std::make_shared<tree_node>();
        node->child_right = std::make_shared<tree_node>();

        assert( best_axis > -1 && best_axis < 3 );

        node->child_left->first_brick_index  = node->first_brick_index;
        node->child_right->first_brick_index = node->first_brick_index;
        node->child_right->first_brick_index[best_axis] = best_pos / brick_size[best_axis];

        node->child_left->num_bricks  = node->num_bricks;
        node->child_left->num_bricks[best_axis] = best_pos / brick_size[best_axis] - node->child_left->first_brick_index[best_axis];

        node->child_right->num_bricks = node->num_bricks;
        node->child_right->num_bricks[best_axis] = node->num_bricks[best_axis] - node->child_left->num_bricks[best_axis];

        node->child_left->bounds = node->bounds;
        node->child_left->bounds.max[best_axis] = best_pos;

        node->child_right->bounds = node->bounds;
        node->child_right->bounds.min[best_axis] = best_pos;


        split(node->child_left.get(),  bs, bd);
        split(node->child_right.get(), bs, bd);
    }
}


//-------------------------------------------------------------------------------------------------
// Traverse leaves and count them
//

void traverse_and_count_leaves(tree_node* node, int& count)
{
    if ( node->child_left == nullptr && node->child_right == nullptr )
    {
        ++count;
    }
    else
    {
        traverse_and_count_leaves( node->child_left.get(), count );
        traverse_and_count_leaves( node->child_right.get(), count );
    }
}


//-------------------------------------------------------------------------------------------------
// Traverse leaves and sample
//

template <typename RandomEngine, typename Distribution01>
void traverse_and_sample_leaves(
        tree_node* node,
        volume_ref const& volume,
        transfer_function_ref const& tf,
        std::vector<sphere>& spheres,
        std::vector<vec4>& colors,
        RandomEngine& engine,
        Distribution01& dist,
        int& prim_id,
        int samples_per_leaf
        )
{
    if ( node->child_left == nullptr && node->child_right == nullptr )
    {
        auto vol_size = vec3(volume.size());
        vol_size /= 0.5f * max_element(vol_size);


        // transform vox -> obj

        auto vox = node->bounds;

        float ox    = ( vox.min.x / static_cast<float>(volume.width()) ) * vol_size.x - vol_size.x / 2.0f;
        float oy    = ( vox.min.y / static_cast<float>(volume.height())) * vol_size.y - vol_size.y / 2.0f;
        float oz    = ( vox.min.z / static_cast<float>(volume.depth()) ) * vol_size.z - vol_size.z / 2.0f;

        vec3 osize  = ( vec3(vox.size()) / vec3(volume.width(), volume.height(), volume.depth()) ) * 2.0f;

        aabb obj( vec3(ox, oy, oz), vec3(ox + osize.x, oy + osize.y, oz + osize.z) );

//      samples_per_leaf = min(samples_per_leaf, (int)visionaray::volume(node->bounds));

        for (int samp = 0; samp < samples_per_leaf; ++samp)
        {
            vec3 rnd( dist(engine), dist(engine), dist(engine) );

            // map to [0..1] texel coordinates
            auto posf = ( rnd * vec3(vox.size()) + vec3(vox.min) )
                      / vec3(volume.width() - 1, volume.height() - 1, volume.depth() - 1);


            // sample volume
            auto raw = tex3D(volume, posf);


            // early out?
//          if (raw == 0) continue;


            sphere s;
            s.center = posf * vol_size - vol_size / 2.0f;
            s.center.y = -s.center.y;
            s.center.z = -s.center.z;
            s.radius = get_radius(obj.size(), samples_per_leaf);
            s.prim_id = prim_id++;
            spheres.push_back(s);
            colors.push_back(tex1D(tf, raw / 255.0f));
        }
    }
    else
    {
        traverse_and_sample_leaves(
                node->child_left.get(),
                volume,
                tf,
                spheres,
                colors,
                engine,
                dist,
                prim_id,
                samples_per_leaf
                );

        traverse_and_sample_leaves(
                node->child_right.get(),
                volume,
                tf,
                spheres,
                colors,
                engine,
                dist,
                prim_id,
                samples_per_leaf
                );
    }
}


//-------------------------------------------------------------------------------------------------
// Importance sampling (dispatch)
//

void sample_volume_importance(
        volume_ref const& volume,
        transfer_function_ref const& tf,
        std::vector<sphere>& spheres,        // vectors will be appended to!
        std::vector<vec4>& colors,           // vectors will be appended to!
        int num_samples
        )
{
    vec3i num_bricks(64, 64, 64);
    auto bs = make_bricks(volume, num_bricks);
    auto bd = make_brick_data(bs, volume);

    tree_node root;
    root.bounds = aabbi(
            vec3i(0, 0, 0),
            vec3i(volume.width(), volume.height(), volume.depth())
            );

    root.first_brick_index = vec3i(0, 0, 0);
    root.num_bricks = bs.count;

    split(&root, bs, bd);

    int count = 0;
    traverse_and_count_leaves(
            &root,
            count
            );


    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);


    int prim_id = 0;
    auto samples_per_leaf = num_samples / count;

    traverse_and_sample_leaves(
            &root,
            volume,
            tf,
            spheres,
            colors,
            engine,
            dist,
            prim_id,
            samples_per_leaf
            );
}


//-------------------------------------------------------------------------------------------------
// TCP server
//

template <typename Buffer>
class server
{
public:

    explicit server(unsigned short port = 31050)
        : manager(makeConnectionManager(port))
    {
    }

    void accept()
    {
        std::cout << "server: accepting...\n";

        manager->accept(boost::bind(&server::handle_new_connection, this, _1, _2));
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
            std::cout << "server: error: " << e.message() << "\n";

            manager->stop();
            return;
        }

        if (reason == Connection::Read)
        {
            // Read complete.
            // Got a message from the client.
            //
            // Nothing to do yet.
        }
        else
        {
            // Write complete.
            // Reload and send data again...

            // Order of messages is MT_Spheres, MT_Colors.

            if (message->type() == MT_Spheres)
            {
                std::cout << "server: sent spheres...\n";

                return;
            }

            if (message->type() == MT_Colors)
            {
                std::cout << "server: sent colors...\n";

                // Blocking :-(
                send_data();
                return;
            }
        }
    }

    bool handle_new_connection(ConnectionPointer connection, boost::system::error_code const& e)
    {
        if (e)
        {
            std::cout << "server: could not connect to client: " << e.message() << "\n";
            return false;
        }

        std::cout << "server: connected.\n";

        // Accept and save this connection
        // and set the message handler of the connection.
        conn = connection;
        conn->set_handler(boost::bind(&server::handle_message, this, _1, _2, _3));

        // Get ready to rumble...
        // Blocking :-(
        send_data();

        return true;
    }

    void set_payload(Buffer const& spheres, Buffer const& colors)
    {
        spheres_ = spheres;
        colors_  = colors;
    }

    void send_data()
    {
        std::cout << "server: loading data...\n";

        load_data();

        std::cout << "server: loading data... done.\n";

        std::cout << "server: sending data...\n";

        write_spheres();
        write_colors();
    }

private:

    bool connected()
    {
        return (bool)conn;
    }

    void write_spheres()
    {
        conn->write(MT_Spheres, spheres_);
    }

    void write_colors()
    {
        conn->write(MT_Colors, colors_);
    }

    bool load_data()
    {
        // load virvo file

        auto filename = "/Users/stefan/visionaray/build/sim3.xvf"; // adapt to your needs

        std::shared_ptr<vvVolDesc> vd = std::make_shared<vvVolDesc>(filename);

        vvFileIO fio;

        std::cout << "    loading data...\n";

        fio.loadVolumeData(vd.get());

        // Set default color scheme if no TF present:
        if (vd->tf[0].isEmpty())
        {
            vd->tf[0].setDefaultAlpha(0, 0.0, 1.0);
            vd->tf[0].setDefaultColors((vd->chan==1) ? 0 : 2, 0.0, 1.0);
        }

        size_t lut_entries = 256;

        std::vector<vec4> tf(lut_entries);

        vd->computeTFTexture(lut_entries, 1, 1, reinterpret_cast<float*>(tf.data()));

        std::cout << "    computing tf texture...\n";

        // convert volume and tf to visionaray texture_refs

        volume_ref vol_tex(vd->vox[0], vd->vox[1], vd->vox[2]);

        vol_tex.set_data(vd->getRaw(0/*frame_num++ % vd->frames*/));
        vol_tex.set_filter_mode(Linear);
        vol_tex.set_address_mode(Clamp);

        transfer_function_ref tf_tex(lut_entries);

        tf_tex.set_data(tf.data());
        tf_tex.set_filter_mode(Linear);
        tf_tex.set_address_mode(Clamp);


        // sample

        std::cout << "    sampling data...\n";

        std::vector<sphere> spheres;
        std::vector<vec4> colors;

        sample_volume_importance(vol_tex, tf_tex, spheres, colors, 100000);

        splatrend::serialize(spheres_, spheres);
        splatrend::serialize(colors_, colors);

        return true;
    }

    ConnectionManagerPointer manager;
    ConnectionPointer conn;

    Buffer spheres_;
    Buffer colors_;

    int frame_num = 0;

};


int main(int argc, char** argv)
{
    try
    {
        server<std::vector<char>> srv;

        srv.accept();
        srv.run();
        srv.wait();

        std::cout << "DONE\n";
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}
