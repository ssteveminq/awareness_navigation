#include "classifier.h"



void printMatrix_(const MatrixXd Matrix_)
{

	for(int i=0;i<Matrix_.rows();i++)
	{
		cout<<"Row : "<<i<<" : ";
		for(int j=0;j<Matrix_.cols();j++)
			cout<<Matrix_(i,j)<<" , ";

		cout<<endl;
	}


}

Classifier::Classifier()
{
	Dim=19;
	ClassNum=3;
	Threshold=0.05;
	IsLearned=false;
}

Classifier::~Classifier()
{


}


int Classifier::Classify( vector<double>& Sample )
{

	return 0;
}

bool Classifier::Learning( vector<vector<double> >& DataSet,vector<double>& DataState )
{
	return false;
}

string* Classifier::StringSplit( string strTarget, string strTok )
{
	int     nCutPos;
	int     nIndex     = 0;
	string* strResult = new string[256];

	while ((nCutPos = strTarget.find_first_of(strTok)) != strTarget.npos)
	{
		if (nCutPos > 0)
		{
			strResult[nIndex++] = strTarget.substr(0, nCutPos);
		}
		strTarget = strTarget.substr(nCutPos+1);
	}

	if(strTarget.length() > 0)
	{
		strResult[nIndex++] = strTarget.substr(0, nCutPos);
	}

	return strResult;
}

int Classifier::SplitString(string str_array[], string strTarget, string strTok )
{

	return 0;
}

RowVectorXd Classifier::getMeanfromList(list<RowVectorXd> datalist )
{
	RowVectorXd  MeanVector;
	int sizeVector=datalist.size();
	
	if(sizeVector<1)
	{
		cout<<"Size error! Check the size of datalist"<<endl;
		exit;
	}

	MeanVector=RowVectorXd(Dim);
	MeanVector.setZero();

	list<RowVectorXd>::iterator Iter= datalist.begin();
	
	for(Iter;Iter != datalist.end();Iter++)
	{

		MeanVector=MeanVector+(*Iter);
	}

	MeanVector=MeanVector/sizeVector;



	return MeanVector;
}


Eigen::RowVectorXd Classifier::getMeanfromMap(const MatrixXd dataMap)
{
	RowVectorXd  MeanVector;
	int sizeVector=dataMap.rows();

	if(sizeVector<1)
	{
		cout<<"Size error! Check the size of Matrix"<<endl;
		exit;
	}

	MeanVector=RowVectorXd(Dim);
	MeanVector.setZero();

	
	for(int i=0;i<sizeVector;i++)
	{
		MeanVector=MeanVector+dataMap.row(i);
	}

	MeanVector=MeanVector/sizeVector;

	return MeanVector;
}

map<int,double> Classifier::FindMaxIndex(const MatrixXd& ProbMatrix_ )
{
	int nrows=ProbMatrix_.rows();
	int ncols=ProbMatrix_.cols();

	double temp=0.0;
	double maxIndex=0.0;
	map<int,double> MaxIndex;
	
	MaxIndex.insert(make_pair(0,ProbMatrix_(0,0)));
	maxIndex=ProbMatrix_(0);

	map<int,double>::iterator mapIter = MaxIndex.begin();

	for(int i=1;i<ncols;i++)
		{
			mapIter = MaxIndex.begin();		
			
			if(mapIter->second<ProbMatrix_(0,i))
			{
				MaxIndex.clear();
				MaxIndex.insert(make_pair(i,ProbMatrix_(0,i)));
				maxIndex=i;
			}
	}

		return MaxIndex;
	
}

vector<string> Classifier::StringSplit2( const char* str, char c/*=','*/ )
{

		vector<string> result;

		do
		{
			const char *begin = str;

			while(*str != c && *str)
				str++;

			result.push_back(string(begin, str));
		} while (0 != *str++);

		return result;
	

}

int Classifier::StringSplit3(char* res_str[], char* src_str, char* delim)
{
	int i=0;

	res_str[i] = strtok(src_str, delim);

	while(res_str[i]!=NULL)
	{
		res_str[++i] = strtok(NULL,delim);
	}
	if (res_str[0]==NULL) return -1;
	return i;
}

ELMClassifier::ELMClassifier()
{
   NeuronNum=8;
   VotingSize=30;
   Dim=19;
   Confidence=0.0;
   
	VotingVector.resize(VotingSize);
	for(int i=0;i<VotingSize;i++)
		VotingVector[i]=0;
}


ELMClassifier::ELMClassifier(int dim, int Num_actions):Classifier()
{
	Dim=dim;
	ClassNum=Num_actions;
	Threshold=0.0;
	IsLearned=false;
}


ELMClassifier::~ELMClassifier()
{

}

int ELMClassifier::Classify( const vector<float> featurevector)
{
	RowVectorXd TestVector;
	TestVector.resize(featurevector.size());

	for(int i(0);i<featurevector.size();i++)
	  TestVector(i)= featurevector[i];

	return Classify(TestVector);
}

int ELMClassifier::Classify(const RowVectorXd TestVector)
{   
	 int		maxindex =0;
	 int		 Aug_ClassNum=ClassNum;
	 int		sigma_length = NeuronNum*(Aug_ClassNum);
	 MatrixXd	H(Dim,sigma_length);
	 MatrixXd	T(Dim,Aug_ClassNum);		 	 
	 RowVectorXd RowMu(Dim);
	 double     Norm_Coeff ;

	 double tempsum=0;
	 double tempdiff=0;
	 double tempp=0.0;

	int count=0;
		 
 
	//TestVector.normalize();
	 for(int i=0;i<sigma_length;i++)
	 { 
		 tempsum=0.0;
		 for(int j=0;j<Dim;j++)
		 {
			tempp= (TestVector(j)-Mu(i,j));
			 tempsum=tempsum+tempp*tempp;
		 }
		    
		 Norm_Coeff=sqrt(tempsum);

		 for(int j=0;j<Dim;j++)
		 {
			 tempdiff= exp(-1*(1/Sigma(i,0))*Norm_Coeff);
			 H(j,i)=tempdiff;
		 }		
	 }

	  T=H*Beta;
	  map<int,double> Maxx;
	  map<int,double>::iterator miter;
	  Maxx = FindMaxIndex(T);
	  miter=Maxx.begin();

	  double tempsss=0;

	  for(int i=0; i<T.cols();i++)
		tempsss=tempsss+T(0,i);

	    tempsss=miter->second/tempsss;
     	maxindex=miter->first+1;
        Confidence=tempsss;
       //cout<< "Confidence is : "<<tempsss<<endl;

     	if( Confidence<0.15)
		    maxindex=0;	

	   // if( tempsss<0.5)
		  //  maxindex=pre_result;
	   // else if(tempsss<0.35)
		  //  maxindex=0;

	   if(maxindex>ClassNum)
		   maxindex=maxindex-ClassNum;
	  
	   pre_result=maxindex;

	 return maxindex;

}

//This funtion uses DatamatrixMap
bool ELMClassifier::Learning()
{

	int index=0;
	int Aug_ClassNum=DataMatrixMap.size();

	cout<<"ClassNum is"<<Aug_ClassNum<<endl;

		
	//Class_Mean Vector
	RowVectorXd MeanVector;
	MatrixXd Class_Mean(Aug_ClassNum,Dim);
	MatrixXd Class_VAR(Aug_ClassNum,Dim);

	// Parameter Initialize
	Class_Mean.setZero();
	Class_VAR.setZero();

	// Size of Mu ( # of class
	int sizeMu=Aug_ClassNum*NeuronNum;
	
	MatrixXd Mu_ind(NeuronNum,Dim);
	Mu.resize(sizeMu,Dim);
 	Sigma.resize(sizeMu,1);

	Mu.setZero();
	Sigma.setZero();
	
	MatrixXd tempMu(NeuronNum,Dim);
	MatrixXd tempSigma(NeuronNum,1);
	MatrixXd tempDiag(Dim,Dim);
		
	//Setting parameter
	for(int k=1;k<Aug_ClassNum+1;k++)
	{
		/*tempdataset=DataListMap[k];*/
		Class_Mean.row(k-1)=getMeanfromMap(DataMatrixMap[k]);
		Class_VAR.row(k-1)=1.5*(DataMatrixMap[k].colwise().maxCoeff()-DataMatrixMap[k].colwise().minCoeff());
			
		tempMu.setZero();
		tempDiag.setZero();
		Mu_ind.setZero();
			for(int j=0;j<NeuronNum;j++)
			{
				Mu_ind.row(j)=Class_Mean.row(k-1);
				//Mu.row(index+j)=Class_Mean.row(k);
			}
				for(int m=0;m<tempMu.rows();m++)
					for(int n=0; n<tempMu.cols();n++)
						tempMu(m,n)=(rand()/((double) RAND_MAX))-0.5;
				
				for(int m=0;m<tempDiag.rows();m++)
					tempDiag(m,m)=Class_VAR.row(k-1)[m];

				
			Mu_ind=Mu_ind+2*tempMu*tempDiag;

			double tempval=0.0;
			for(int m=0;m<tempMu.rows();m++)
			{	
				tempval=2*Class_VAR.row(k-1).maxCoeff();
				tempSigma(m,0)=tempval;

			}

			index=(k-1)*NeuronNum;
		
			Mu.middleRows(index,Mu_ind.rows())=Mu_ind;
			Sigma.middleRows(index,tempSigma.rows())=tempSigma;
	}

	MatrixXd columVec;
	MatrixXd tempMatrix;
	MatrixXd tempMatrix_T;
	MatrixXd T;
	map<int,MatrixXd>::iterator Datamapiter =DataMatrixMap.begin();
		
	T.resize((Datamapiter->second).rows(),Aug_ClassNum);
	T.setZero();
	columVec.resize((Datamapiter->second).rows(),1);
	columVec.setOnes();

	T.col(0)= columVec;

	DataMatrixforLerning=Datamapiter->second;
	Datamapiter++;

	int rowsize=DataMatrixforLerning.rows();
	int colsize=DataMatrixforLerning.cols();
	int prerowindex=0;

	for(Datamapiter;Datamapiter!=DataMatrixMap.end();Datamapiter++)
	{
		tempMatrix=DataMatrixforLerning;
		prerowindex=rowsize;

		rowsize=rowsize+(Datamapiter->second).rows();
		
		DataMatrixforLerning.resize(rowsize,colsize);
		DataMatrixforLerning.middleRows(0,prerowindex)=tempMatrix;
		DataMatrixforLerning.middleRows(prerowindex,(Datamapiter->second).rows())=Datamapiter->second;
		
		tempMatrix_T=T;

		T.resize(rowsize,Aug_ClassNum);
		T.setZero();
		T.middleRows(0,prerowindex)=tempMatrix_T;
		
		columVec.resize((Datamapiter->second).rows(),1);
		columVec.setOnes();
		
		T.block(prerowindex,Datamapiter->first-1,(Datamapiter->second).rows(),1)=columVec;
	}

	MatrixXd H(DataMatrixforLerning.rows(),sizeMu);
	MatrixXd Norm_Coeff(DataMatrixforLerning.rows(),1);
		
	double tempp=0.0;
	double tempsum=0.0;
	double tempdiff=0.0;

 	for(int i=0;i<sizeMu;i++){ 
		MatrixXd TempH(DataMatrixforLerning.rows(),1);
		TempH.setOnes();
		TempH=TempH*Mu.row(i);
		TempH=DataMatrixforLerning-TempH;

		for(int k=0;k<TempH.rows();k++){
			tempsum=0.0;

			for(int j=0;j<TempH.cols();j++){
				tempp= TempH(k,j);
  				tempsum=tempsum+tempp*tempp;
 			}

				tempsum  = sqrt(tempsum);
				tempdiff = exp(-1*(1/Sigma(i,0))*tempsum);
				H(k,i)=tempdiff;
		}

 	}

	MatrixXd  pH=pseudoInverse__(H);
	Beta=pH*T;
	IsLearned=true;

	//Print Mu // Sigma // Beta : Learning parameters
	//Mu
	//Printf Mu, sigma, Beta --Learning parameters
	cout<<"-----MU-----"<<endl;
	for(int x=0;x<Mu.rows();x++)
	{
		for(int y=0;y<Mu.cols();y++)
			cout<<Mu(x,y)<<" , ";
			cout<<endl;
	}

	cout<<"-----Sigma-----"<<endl;
	for(int x=0;x<Sigma.rows();x++)
	{
		for(int y=0;y<Sigma.cols();y++)
			cout<<Sigma(x,y)<<" , " ;
			cout<<endl;
	}

	cout<<"-----Beta-----"<<endl;
		for(int x=0;x<Beta.rows();x++)
	{
		for(int y=0;y<Beta.cols();y++)
			cout<<Beta(x,y)<<" , ";
			cout<<endl;
	}

	return true;
}

bool ELMClassifier::readDataParameters()
{
	int sigma_length = NeuronNum*ClassNum;
	cout<<sigma_length<<endl;
	
	//Sigma Read
	ifstream InputFile;
	string nameofFile="../../mu.txt";
	InputFile.open(nameofFile.c_str());
	
	Sigma= VectorXd(sigma_length);
	
	if(!InputFile.is_open()){
		cout << "file open error... check the txt file" << endl;
		exit(0);
	}

	string str;
	char spell[256]; 
	int iter=0;
	float b ;

	while(!InputFile.eof())
	{
		if(iter>sigma_length-1)
		break;

		InputFile.getline(spell, 1028);				//한줄 읽어오기   
		//for(int i = 0; i<21000; i++)
		str=spell;
		str.erase(str.length(),1) ;
		b = atof(str.c_str()) ;
		Sigma(iter)=b;
		iter++;
	}

	
	InputFile.close();
	// Mu Read
	ifstream InputFile2;
	nameofFile="../../mu.txt";
	InputFile2.open(nameofFile.c_str());
	Mu= MatrixXd(sigma_length,Dim);

	if(!InputFile2.is_open()){
		cout << "mu file open error... check the txt file" << endl;
		exit(0);
	}

	 iter=0;
	 string * tok;
	 char toki[1] = {' '};

	while(!InputFile2.eof())
	{
		if(iter>sigma_length-1)
			break;

		InputFile2.getline(spell, 1028);				//한줄 읽어오기   
		//for(int i = 0; i<21000; i++)
		
		StringTokenizer sts=StringTokenizer(spell);
		tok = StringSplit(spell,toki);
		
		for(int j=0;j<Dim;j++)
		{
			str=sts.nextToken();
			//str=tok[j];
			str.erase(str.length(),1) ;
			b = atof(str.c_str()) ;
			Mu(iter,j)=b;
			
		}

		iter++;
	}

	InputFile2.close();		
	
	// Beta Read
	ifstream InputFile3;
	string Betaname="../../Beta.txt";
	InputFile3.open(Betaname.c_str());
	Beta= MatrixXd(sigma_length,ClassNum);

	if(!InputFile3.is_open()){
		cout << "Beta file open error... check the txt file" << endl;
		exit(0);
	
	}
	iter=0;
	
	while(!InputFile3.eof())
	{
		if(iter>sigma_length-1)
			break;

		InputFile3.getline(spell, 1024);				//한줄 읽어오기   

		StringTokenizer st=StringTokenizer(spell);
		//for(int i = 0; i<21000; i++)

		tok = StringSplit(spell,toki);

		for(int j=0;j<ClassNum;j++)
		{
			//str=tok[j];
			str=st.nextToken();
			str.erase(str.length(),1) ;
			b = atof(str.c_str()) ;
			Beta(iter,j)=b;

		}

		iter++;
	}

	InputFile3.close();		

}	


bool ELMClassifier::readSignalData()
{
	int j=0;
	int temps=0;
	int MaxBuffer=3000;
	
	DataSet= MatrixXd(MaxBuffer,Dim);
	DataSetState= MatrixXd(MaxBuffer,1);

	RowVectorXd tempDataVec(Dim);

	if(!InputFile.is_open()){
		cout << "Data file load error... check the data file" << endl;
		exit(0);
	}

	string str;
	char spell[150]; 
	int iter=0;
	float b ;
	int classState=0;
	string * tok;
	char toki[1] = {' '};

	while(!InputFile.eof())
	{
		if(iter>MaxBuffer-1)
			break;

		InputFile.getline(spell, 150);				//한줄 읽어오기   
		//for(int i = 0; i<21000; i++)

		tok = StringSplit(spell,toki);

		if (tok[9] == "" && tok[0] !="")
		{
			for(j=0;j<Dim;j++)
			{
				str=tok[j];
				str.erase(str.length(),1) ;
				b = static_cast<double>(atof(str.c_str())) ;
				DataSet(iter,j)=b;
				tempDataVec[j]=b;
			}
		
			str=tok[j];
			str.erase(str.length(),1) ;
			classState = static_cast<int>(atof(str.c_str()));
			DataSetState(iter,0)=classState;
		

			if(classState>0)
				{
					//double tempmax =max(tempDataVec);
					if(Threshold<tempDataVec.maxCoeff())
						DataListMap[classState].push_back((tempDataVec));
				}
		}

		iter++;
	}

	map<int,list<RowVectorXd> >::iterator mapiter=DataListMap.begin();
	
	for(mapiter;mapiter!=DataListMap.end();mapiter++)
	{
		int datasizerow= mapiter->second.size();
		MatrixXd		tempMatrix(datasizerow,Dim);
		list<RowVectorXd>::iterator tempiter =mapiter->second.begin();

		int jj=0;
		for(tempiter; tempiter!=mapiter->second.end();tempiter++)
		{
			tempMatrix.row(jj)=*tempiter;
			//cout<<j<<endl;
			jj++;
			
		}
	
		DataMatrixMap[mapiter->first]=tempMatrix;

		}

	ODataMatrixMap=DataMatrixMap;


	InputFile.close();	
	IsLearned=true;
}

void ELMClassifier::offlineSimulate()
{

	int DataSize =DataMatrixforLerning.rows();
	int ResultClass=0;
	double accuracy=0.0;
	int count=0;
	int answer=0;

	ClassNum=3;
	std::vector<int> resultvec;
//	RowVectorXd TestVector;

	map<int,MatrixXd>::iterator mapiter=DataMatrixMap.begin();

	for(mapiter; mapiter!=DataMatrixMap.end();mapiter++)
	{
		
		for(int j=0;j<mapiter->second.rows();j++)
		{

			ResultClass=Classify(mapiter->second.row(j));

			if(ResultClass>ClassNum)
				ResultClass=ResultClass-ClassNum;

			if(mapiter->first>ClassNum)
				answer=mapiter->first-ClassNum;			


			if((ResultClass==mapiter->first))
				count++;

			resultvec.push_back(ResultClass);
		}
	}
	
	accuracy=((double)count/(double)DataSize);
	
	cout<<"accuracy :" <<accuracy <<endl;

}

bool ELMClassifier::UpdateLearningParameters(vector< vector<float> >_DataSet, vector<int> DataState)
{
	// map<int,MatrixXd>::iterator datamapiter;

	// if(_DataSet.size()==DataState.size())
	// {
	// 	MatrixXd tempMatrix;
	// 	for(int i(0);i<_DataSet.size();i++)
	// 		datamapiter=DataMatrixMap.find(DataState[i]);
			
	// 		tempMatrix.resize(datamapiter->second.rows()+1,datamapiter->second.cols());
	// 		//tempMatrix=datamapiter->second;

	// 		for(int m(0);m<(datamapiter->second).rows();m++)
	// 			for(int n(0);n<tempMatrix.cols();n++)
	// 				tempMatrix(m,n)= (datamapiter->second)(m,n);

	// 			for(int k(0);k<tempMatrix.cols();k++)
	// 				tempMatrix(datamapiter->second.rows()+1,k)=DataState[k];

	// 	  datamapiter->second=tempMatrix;	
	// }
	// else
	// 	cout<<"Data size is wrong"<<endl;


 return true;
}

int ELMClassifier::ResultVoting( int cmd )
{
	
	int count=1;
	vector<int> countvec;
	countvec.resize(ClassNum+1);

		for(int i=0; i< VotingSize-1;i++){

			VotingVector[i]=VotingVector[i+1];
		}

		VotingVector[VotingSize-1]=cmd;

		for (int j=0; j<ClassNum+1;j++)
			countvec[j]=std::count(VotingVector.begin(),VotingVector.end(),j);

		//auto it=std::max_element(countvec.begin(),countvec.end());
		cmd =std::distance(countvec.begin(), std::max_element(countvec.begin(),countvec.end()));
		return cmd;
}

int ELMClassifier::readSignalDataFile()
{
		int j=0;
		int temps=0;
		//std::string FileName = "/home/mk/catkin_ws/src/hri_final_project/Classifier/src/Dataset.csv";
		std::string FileName = "/home/mk/TrainingData_.csv";
		ifstream InputFile(FileName.c_str());

		string str;
		RowVectorXd tempDataVec;

		bool    columncheck=false;
		char 	spell[150]; 
		int 	iter=0;
		float 	b ;
		int 	classState=0;
		int 	absIndex=-1, pre_absIndex=-1;
		char 	toki[1] = {','};
		char 	tok2=',';
		char 	*wow[20];
		char 	*strtokens[5];
		int 	i;
		int 	res;
		int 	DataLenth=0;

		////// -- Check the Data Lenght -- ////
		if(!InputFile.is_open()){
			cout << "Data file load error... check the data file" << endl;
			exit(0);
		}
		else
		{
			cout<<"Success"<<endl;
			while(!InputFile.eof())
			{
				InputFile.getline(spell, 150);				//
				if (spell[0]=='\0')					//empty line
					continue;
				DataLenth++;
				cout<<spell[0]<<endl;
			}
		}

		cout<<"Data Number : "<<DataLenth<<endl;
		InputFile.close();	


		//////Read DataSetState////
		std::string FileName2 = "/home/mk/TrainingState_.csv";
		ifstream InputFile2(FileName2.c_str());

		DataSetState= MatrixXd(DataLenth,1);
		
		//InputFile.open(FileName.c_str());
		if(!InputFile2.is_open()){
			cout << "DataState file load error..check file" << endl;
			exit(0);
		}
		else
		{
			iter=0;
			cout<<" Reading Data State File"<<endl;

			while(!InputFile2.eof())
			{
				InputFile2.getline(spell, 50);				//
				if (spell[0]=='\0')							//empty line
					continue;

				str=strtokens[0] = strtok(spell,",");
				str.erase(str.length(),1);
				classState = static_cast<int>(atof(str.c_str()));
				DataSetState(iter,0)=classState;
				iter++;
			}
		}

		InputFile2.close();
		
		////// -- Read Dataset --////
		DataListMap.clear();
		DataSet= MatrixXd(DataLenth,Dim);
		InputFile.open(FileName.c_str());
		if(!InputFile.is_open()){
			cout << "Data file load error... check the data file" << endl;
			exit(0);
		}
		else{
			iter=0;
			while(!InputFile.eof()) 
			{
				InputFile.getline(spell, 100);				

				if (spell[0]=='\0')		//empty line
					continue;

				i=0;
				wow[i] = strtok(spell,",");
				while(wow[i]!=NULL)
					wow[++i] = strtok(NULL,",");
				
				Dim=i;

				//cout<<"Feature dimension is"<<Dim<<endl;
				tempDataVec.resize(Dim);
				tempDataVec.setZero();

				for(j=0;j<Dim;j++)
				{
					str=wow[j];
					str.erase(str.length(),1) ;
					b = static_cast<float>(atof(str.c_str())) ;
					DataSet(iter,j)=b;
					tempDataVec[j]=b;

				}

				//get relative index
				int classState=DataSetState(iter,0);
				if(classState>0)
				{
					DataListMap[classState].push_back(tempDataVec);
				}
				iter++;
			} 

		}

	   InputFile.close();	


	   //Save from ListMap to MatrixMap
		setNumofClass((int)DataListMap.size());
		map<int,list<RowVectorXd> >::iterator mapiter=DataListMap.begin();

		for(mapiter;mapiter!=DataListMap.end();mapiter++)
		{
			int datasizerow= mapiter->second.size();
			MatrixXd	tempMatrix(datasizerow,Dim);
			list<RowVectorXd>::iterator tempiter =mapiter->second.begin();

			int jj=0;
			for(tempiter; tempiter!=mapiter->second.end();tempiter++)			//FIXME: this make error in re-loading 
			{
				tempMatrix.row(jj)=*tempiter;
				//cout<<j<<endl;
				jj++;

			}

			DataMatrixMap[mapiter->first]=tempMatrix;

		}

		 ODataMatrixMap=DataMatrixMap;

		 //Save Original Data to TotalTrainingDataset
		




		//Print Matrix
		 cout<<"Print Matrix in reading"<<endl;
		for(int k(0);k<DataSet.rows();k++)
		{ 
				 for(int l(0);l<DataSet.cols();l++)
		 		cout<<DataSet(k,l)<<",";

		 	cout<<endl;
		 }

		map<int,MatrixXd>::iterator int_mapiters=DataMatrixMap.begin();
		for(int_mapiters;int_mapiters!=DataMatrixMap.end();int_mapiters++)
		{
			cout<<"---------"<< int_mapiters->first<<"--------"<<endl;


		int rowsize=int_mapiters->second.rows();
		int colsize=int_mapiters->second.cols();

			for(int k(0);k<rowsize;k++)
			{
				for(int l(0);l<colsize;l++)
					cout<<(int_mapiters->second)(k,l)<<" , ";

				cout<<endl;	

			}
		 }

		return Dim;
}

//it add to the previous dataset (list) 
void ELMClassifier::Updatedatalistmap( vector <vector<float> >_DataSet, vector<int> DataState)
{

		RowVectorXd tempDataVec;
		int dim_=_DataSet[0].size();
		tempDataVec.resize(dim_);

		//Size must be same
		if(_DataSet.size()==DataState.size())
	 	{		
			for(int i(0);i<_DataSet.size();i++)
			 {
			 	
			 	tempDataVec.setZero();
			 	for(int j(0);j<_DataSet[0].size();j++)
			 	{	
			 		tempDataVec(j)=	_DataSet[i][j];
			 		//cout<<"tempvec"<<tempDataVec(j)<<",";
			 	}

			 	cout<<endl;
			 	DataListMap[DataState[i]].push_back(tempDataVec);
	           	cout<<"state :"<<DataState[i]<<endl;

			 }

		}


}

		//  	cout<<endl;
		//  }

		// map<int,MatrixXd>::iterator int_mapiters=DataMatrixMap.begin();
		
void ELMClassifier::updateMatrixMap()
{

	DataMatrixMap.clear();
	setNumofClass((int)DataListMap.size());
	map<int,list<RowVectorXd> >::iterator mapiter=DataListMap.begin();

	RowVectorXd DD;
	cout<<"Dimension is "<<Dim<<endl;

	for(mapiter;mapiter!=DataListMap.end();mapiter++)
	{
		int datasizerow= mapiter->second.size();
		cout<<"datasize row"<<datasizerow<<endl;
		MatrixXd	tempMatrix(datasizerow,Dim);
		tempMatrix.setZero();
		list<RowVectorXd>::iterator tempiter =mapiter->second.begin();

		int row_idx=0;
		for(tempiter;tempiter!=mapiter->second.end();tempiter++)
		{
			
			for(int k=0;k<Dim;k++)
			{	
				tempMatrix(row_idx,k)=(*tempiter)[k];
				cout<<tempMatrix(row_idx,k)<<",";
			}	
			cout<<endl;
			row_idx++;
		}

		 DataMatrixMap[mapiter->first]=tempMatrix;

		}
		// int jj=0;
		// for(int (i); i<datasizerow;i++)			//FIXME: this make error in re-loading 
		// {
		// 	cout<<"jj"<<jj<<endl;
		// 	RowVectorXd DD=(*tempiter);
			
		// 	for(int m=0;m<Dim;m++)
		// 		cout<<DD(m)<<"DD"<<endl;
			
		// 	// for(int k=0;k<Dim;k++)
		// 	// 	cout<<(*tempiter)[k]<<" ,";
			
		// 		//cout<<endl;
		// 	//tempMatrix(u,k)=(*tempiter)[k];
		
		// 	jj++;
		// 	}

		// DataMatrixMap[mapiter->first]=tempMatrix;

		// }



	
}
