#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <map>
#include <list>
#include "StringTokenizer.h"
#include <iterator>
#include <algorithm>

using namespace Eigen;
using namespace std;

void printMatrix_(const MatrixXd Matrix_);

class Classifier{

public:
	Classifier();
	virtual ~Classifier();
	
	int    Dim;
	int    ClassNum;
	double Threshold;
	bool   IsLearned;
	virtual bool Learning (vector<vector<double> >& DataSet,vector<double>& DataState);
	virtual int Classify (vector<double>& Sample);
	
	map<int,double> FindMaxIndex(const MatrixXd& ProbMatrix_);
	map<int,list<RowVectorXd> > DataListMap;

	map<int,MatrixXd> 	ODataMatrixMap;
	map<int,MatrixXd>	DataMatrixMap;
	MatrixXd 			DataMatrix;
	MatrixXd 			DataMatrixforLerning;

	int 			SplitString(string str_array[], string strTarget, string strTok );
	vector<string>  StringSplit2( const char* str, char c/*=','*/ );
	int 		    StringSplit3(char* res_str[],  char* src_str, char* delim );
	string*     	StringSplit(string strTarget, string strTok);
	RowVectorXd 	getMeanfromList( const list<RowVectorXd> datalist);
	RowVectorXd 	getMeanfromMap( const MatrixXd dataMap);
	double      	getThreshold(){return Threshold;}
	
};

class ELMClassifier: public Classifier{

public:
	ELMClassifier();
	ELMClassifier(int dim, int Num_actions);
	virtual ~ELMClassifier();

	void 		setThreshold(double threshold_){Threshold=threshold_;};

	virtual bool Learning ();
	virtual int  Classify(const RowVectorXd TestVector);
			int  Classify(const vector<float> featurevector);

	void 		setDimension(int Dim_){Dim=Dim_;}
	void 		setNumofClass(int NumofClass_){ClassNum=NumofClass_;}
	void		offlineSimulate();
	bool		readDataParameters();
	bool		readSignalData();
	int			readSignalDataFile();
	int 		ResultVoting(int cmd);
	void        Updatedatalistmap( vector <vector<float> >_DataSet, vector<int> DataState);
    void        updateMatrixMap();
	bool 		UpdateLearningParameters(vector< vector<float> >_DataSet, vector<int> DataState);

	int			NeuronNum;
	int 		pre_result;
	int 		VotingSize;
	double 		Confidence;

	ifstream	InputFile;
	MatrixXd	W_projection;
	VectorXd	Sigma;
	MatrixXd	Mu;
	MatrixXd	Beta;
	MatrixXd	DataSet;
	MatrixXd	DataSetState;

    std::vector<int> 		    TrainingDataState;    
	std::vector<int> 			VotingVector;
	std::vector<vector<float> > TrainingDataSet;       
	
	
};



 template<typename _Matrix_Type_>
	 _Matrix_Type_ pseudoInverse__(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
	 {
	     Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU |Eigen::ComputeThinV);
		double tolerance = epsilon * max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
		return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(),0).matrix().asDiagonal() * svd.matrixU().adjoint();
 }

	 template<typename Scalar>
	 bool pinv(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &a, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &a_pinv)
	 {
		 //// see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method

		 if ( a.rows()<a.cols() )
			 return false;

		 // SVD
		 Eigen::JacobiSVD< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > svdA(a);

		 Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> vSingular = svdA.singularValues();

		 // Build a diagonal matrix with the Inverted Singular values
		 // The pseudo inverted singular matrix is easy to compute :
		 // is formed by replacing every nonzero entry by its reciprocal (inversing).
		 Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Eigen::RowMajor> vPseudoInvertedSingular(svdA.matrixV().cols(),1);

		 for (int iRow =0; iRow<vSingular.rows(); iRow++)
		 {
			 if ( fabs(vSingular(iRow))<=1e-10 ) // Todo : Put epsilon in parameter
			 {
				 vPseudoInvertedSingular(iRow,0)=0.;
			 }
			 else
			 {
				 vPseudoInvertedSingular(iRow,0)=1./vSingular(iRow);
			 }
		 }

		 // A little optimization here
		 Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mAdjointU = svdA.matrixU().adjoint().block(0,0,vSingular.rows(),svdA.matrixU().adjoint().cols());

		 // Pseudo-Inversion : V * S * U'
		 a_pinv = (svdA.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * mAdjointU  ;

		 return true;
	 }



