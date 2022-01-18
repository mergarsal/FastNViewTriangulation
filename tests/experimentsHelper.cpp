#include "experimentsHelper.h"



template <typename Scalar=double>
void readSetParams(ifstream & in_file, int & N, vector<Scalar> & arr)
{
        
        string line_i, line2; 
        
        // discard first line (comment)
        getline(in_file, line_i);
        // get param
        getline(in_file, line_i);
        // get nine components
        std::stringstream ss(line_i);
        
        getline(ss, line2, ',');
        N = (int)atof(line2.c_str());

        // clear arr if it is not empty 
        if (!arr.empty()) arr.clear();
        
        for (size_t j_col = 0; j_col < N; j_col++)
            {
              getline(ss, line2, ',');
              arr.push_back((double)atof(line2.c_str()));
            }
            
        // show
        std::cout << "Set of params read:\n";
        for (size_t j_col = 0; j_col < N; j_col++)
            {
              std::cout << arr[j_col] << " "; 
            }
        std::cout << std::endl;
        
}



bool readOptionsFromFile(string name_file, 
                         SceneOptions & options)
{

        // aux. strings
        string line_i, line2;


        // try to open the file 
        // open file
        ifstream in_file(name_file);

        // check if file exists
        if (!in_file.good()) return false;
        
        
        /* READ PARAMETERS !!! */ 
        
        /* MAXIMUM NUMBER OF ITERATIONS */ 
        // discard first line (comment)
        getline(in_file, line_i);
        // get param
        getline(in_file, line_i);
        options.max_iter = (double)atof(line_i.c_str());
        std::cout << "Maximum number of iterations: " << options.max_iter << std::endl;
        
        /* Number points per problem instance */ 
        // discard first line (comment)
        getline(in_file, line_i);
        // get param
        getline(in_file, line_i);
        options.n_points = (int)atof(line_i.c_str());
        std::cout << "Number points per problem instance: " << options.n_points << std::endl;

        /* Image size */
        std::cout << "Number cameras\n";
        readSetParams<double>(in_file, options.n_arr_cams, options.arr_cams);
        
        
        /* MAXIMUM PARALLAX */
        std::cout << "Maximum parallax:\n";
        readSetParams<double>(in_file, options.n_parallax, options.max_parallax);
            
        /* FOCAL LENGTH */
        std::cout << "Focal length:\n";
        readSetParams<double>(in_file, options.n_focal, options.focal_length);
        
        /* NOISE */
        std::cout << "Noise:\n";
        readSetParams<double>(in_file, options.n_noise, options.noise);
                   
        /* NUMBER OF CORRESPONDENCES */
        std::cout << "Distance to center:\n";
        readSetParams<double>(in_file, options.n_dist_centers, options.dist_centers);
            
    
        /* TRANSLATION */ 
        // discard first line (comment)
        getline(in_file, line_i);
        // get param
        getline(in_file, line_i);
        options.use_custom_t = (bool)atof(line_i.c_str());
        std::cout << "Use custom t:\n" << options.use_custom_t << std::endl;
        
        /* Read translation vector */ 
        std::cout << "Translation vector:\n";
        int dum_var;
        std::vector<double> temp_t; 
        readSetParams<double>(in_file, dum_var, temp_t);
        
        for (int i = 0; i < dum_var; i++)
                options.custom_t(i) = temp_t[i];
        
        
        /* ROTATION */ 
        // discard first line (comment)
        getline(in_file, line_i);
        // get param
        getline(in_file, line_i);
        options.max_rotation = (double)atof(line_i.c_str());
        std::cout << "Max rotation:\n" << options.max_rotation << std::endl;
        
        /* TRANSLATION SELECTION */ 
        // discard first line (comment)
        getline(in_file, line_i);
        // get param
        getline(in_file, line_i);
        options.method_trans = (int)atof(line_i.c_str());
        std::cout << "Pose generation:\n" << options.method_trans << std::endl;
         
         
        /* Perturbation angle for rotation */ 
        // discard first line (comment)
        getline(in_file, line_i);
        // get param
        getline(in_file, line_i);
        options.noise_rot = (double)atof(line_i.c_str());
        std::cout << "Noise rotation:\n" << options.noise_rot << std::endl;
        
        /* Perturbation angle for translation */ 
        // discard first line (comment)
        getline(in_file, line_i);
        // get param
        getline(in_file, line_i);
        options.noise_trans = (double)atof(line_i.c_str());
        std::cout << "Translation rotation:\n" << options.noise_trans << std::endl;
        
        
        // Close the file
        in_file.close();
        
        
        return true;
}


