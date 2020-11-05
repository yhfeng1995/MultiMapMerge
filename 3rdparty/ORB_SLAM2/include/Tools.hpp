#ifndef _TOOLS_HPP_
#define _TOOLS_HPP_

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

class Tools
{
public:
    //! load & save cv::Mat
    static void SaveCvMat(std::fstream& f, const cv::Mat& mat)
    {
        int matrix_type = mat.type();
        int cols = mat.cols;
        int rows = mat.rows;
        unsigned long int elemsize = mat.elemSize()*cols*rows;
        f.write((char*)&matrix_type, sizeof(matrix_type));
        f.write((char*)&cols, sizeof(cols));
        f.write((char*)&rows, sizeof(rows));
        f.write((char*)&elemsize, sizeof(elemsize));
        f.write((char*)mat.data, elemsize);
    }

    static void LoadCvMat(std::fstream& f, cv::Mat& out)
    {
        int matrix_type;
        int cols;
        int rows;
        unsigned long int elemsize;
        f.read((char*)&matrix_type, sizeof(matrix_type));
        f.read((char*)&cols, sizeof(cols));
        f.read((char*)&rows, sizeof(rows));
        f.read((char*)&elemsize, sizeof(elemsize));

        cv::Mat mat = cv::Mat::zeros(rows,cols,matrix_type);

        // std::cout << "cols: " << cols << "rows" << rows << std::endl;
        // std::cout << mat.elemSize() << std::endl;
        // std::cout << elemsize << std::endl;
        assert(elemsize == mat.elemSize()*cols*rows);

        f.read((char*)mat.data, elemsize);
        //std::cout << mat << std::endl;

        mat.copyTo(out);
    }

    //! load & save DBow2::BowVector
    static void SaveBowVector(std::fstream& f, DBoW2::BowVector& bow_vector)
    {
        unsigned long int dbowvec_size = bow_vector.size();
        f.write((char*)&dbowvec_size, sizeof(dbowvec_size));
        for(auto irit = bow_vector.begin(); irit != bow_vector.end(); ++irit)
        {
            DBoW2::WordId  nodeid = irit->first;
            DBoW2::WordValue weight = irit->second;

            f.write((char*)&nodeid, sizeof(nodeid));
            f.write((char*)&weight, sizeof(weight));
        }
    }

    static void LoadBowVector(std::fstream& f, DBoW2::BowVector& bow_vector)
    {
        bow_vector.clear();

        unsigned long int dbowvec_size;
        f.read((char*)&dbowvec_size, sizeof(dbowvec_size));
        for(unsigned long int i=0; i<dbowvec_size; i++)
        {
            DBoW2::WordId  nodeid=0;
            DBoW2::WordValue weight=0;

            f.read((char*)&nodeid, sizeof(nodeid));
            f.read((char*)&weight, sizeof(weight));
            bow_vector.addWeight(nodeid, weight);
        }
    }

    //! load & save DBow2::FeatureVector
    static void SaveFeatureVector(std::fstream& f, DBoW2::FeatureVector& feat_vector)
    {
        unsigned long int featvec_size = feat_vector.size();
        f.write((char*)&featvec_size, sizeof(featvec_size));
        for(auto drit = feat_vector.begin(); drit != feat_vector.end(); ++drit)
        {
            DBoW2::NodeId nid = drit->first;
            std::vector<unsigned int> features = drit->second;

            // save info of last_nid
            f.write((char*)&nid, sizeof(nid));

            unsigned long int fsize = features.size();
            f.write((char*)&fsize, sizeof(fsize));
            for(std::vector<unsigned int>::iterator i = features.begin(); i != features.end(); ++i)
            {
                unsigned int &fid = *i;
                f.write((char*)&fid, sizeof(fid));
            }
        }
    }

    static void LoadFeatureVector(std::fstream& f, DBoW2::FeatureVector& feat_vector)
    {
        feat_vector.clear();

        unsigned long int featvec_size = 0;
        f.read((char*)&featvec_size, sizeof(featvec_size));
        for(unsigned long int i=0; i<featvec_size; i++)
        {
            DBoW2::NodeId nid = 0;
            std::vector<unsigned int> features;

            f.read((char*)&nid, sizeof(nid));

            unsigned long int fsize = 0;
            f.read((char*)&fsize, sizeof(fsize));
            for(unsigned long int j=0; j<fsize; j++)
            {
                unsigned int fid = 0;
                f.read((char*)&fid, sizeof(fid));

                feat_vector.addFeature(nid, fid);
            }
        }
    }
};

#endif