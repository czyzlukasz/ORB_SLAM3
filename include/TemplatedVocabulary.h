/**
 * This is a modified version of TemplatedVocabulary.h from DBoW2 (see below).
 * Added functions: Save and Load from text files without using cv::FileStorage.
 * Date: August 2015
 * Raúl Mur-Artal
 */

/**
 * File: TemplatedVocabulary.h
 * Date: February 2011
 * Author: Dorian Galvez-Lopez
 * Description: templated vocabulary
 * License: see the LICENSE.txt file
 *
 */

#pragma once

#include <DBoW2/TemplatedVocabulary.h>

namespace ORB_SLAM3 {

/// @param TDescriptor class of descriptor
/// @param F class of descriptor functions
template<class TDescriptor, class F>
/// Generic Vocabulary
struct TemplatedVocabulary : DBoW2::TemplatedVocabulary<TDescriptor, F>
{
  using Base = ::DBoW2::TemplatedVocabulary<TDescriptor, F>;
  /**
   * Loads the vocabulary from a text file
   * @param filename
   */
  bool loadFromTextFile(const std::string &filename);

  /**
   * Saves the vocabulary into a text file
   * @param filename
   */
  void saveToTextFile(const std::string &filename) const;
};

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
bool TemplatedVocabulary<TDescriptor,F>::loadFromTextFile(const std::string &filename)
{
  std::ifstream f;
  f.open(filename.c_str());

  if(f.eof())
    return false;

  Base::m_words.clear();
  Base::m_nodes.clear();

  std::string s;
  getline(f,s);
  std::stringstream ss;
  ss << s;
  ss >> Base::m_k;
  ss >> Base::m_L;
  int n1, n2;
  ss >> n1;
  ss >> n2;

  if(Base::m_k<0 || Base::m_k>20 || Base::m_L<1 || Base::m_L>10 || n1<0 || n1>5 || n2<0 || n2>3)
  {
    std::cerr << "Vocabulary loading failure: This is not a correct text file!" << std::endl;
    return false;
  }

  Base::m_scoring = (::DBoW2::ScoringType)n1;
  Base::m_weighting = (::DBoW2::WeightingType)n2;
  Base::createScoringObject();

  // nodes
  int expected_nodes =
          (int)((pow((double)Base::m_k, (double)Base::m_L + 1) - 1)/(Base::m_k - 1));
  Base::m_nodes.reserve(expected_nodes);

  Base::m_words.reserve(pow((double)Base::m_k, (double)Base::m_L + 1));

  Base::m_nodes.resize(1);
  Base::m_nodes[0].id = 0;
  while(!f.eof())
  {
    std::string snode;
    getline(f,snode);
    std::stringstream ssnode;
    ssnode << snode;

    int nid = Base::m_nodes.size();
    Base::m_nodes.resize(Base::m_nodes.size()+1);
    Base::m_nodes[nid].id = nid;

    int pid ;
    ssnode >> pid;
    Base::m_nodes[nid].parent = pid;
    Base::m_nodes[pid].children.push_back(nid);

    int nIsLeaf;
    ssnode >> nIsLeaf;

    std::stringstream ssd;
    for(int iD=0;iD<F::L;iD++)
    {
      std::string sElement;
      ssnode >> sElement;
      ssd << sElement << " ";
    }
    F::fromString(Base::m_nodes[nid].descriptor, ssd.str());

    ssnode >> Base::m_nodes[nid].weight;

    if(nIsLeaf>0)
    {
      int wid = Base::m_words.size();
      Base::m_words.resize(wid+1);

      Base::m_nodes[nid].word_id = wid;
      Base::m_words[wid] = &Base::m_nodes[nid];
    }
    else
    {
      Base::m_nodes[nid].children.reserve(Base::m_k);
    }
  }

  return true;

}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::saveToTextFile(const std::string &filename) const
{
  std::fstream f;
  f.open(filename.c_str(),std::ios_base::out);
  f << Base::m_k << " " << Base::m_L << " " << " " << Base::m_scoring << " " << Base::m_weighting << std::endl;

  for(size_t i=1; i<Base::m_nodes.size();i++)
  {
    const typename Base::Node& node = Base::m_nodes[i];

    f << node.parent << " ";
    if(node.isLeaf())
      f << 1 << " ";
    else
      f << 0 << " ";

    f << F::toString(node.descriptor) << " " << (double)node.weight << std::endl;
  }

  f.close();
}

} // namespace ORB_SLAM3
