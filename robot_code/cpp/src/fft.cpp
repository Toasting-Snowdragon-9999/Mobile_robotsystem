#include "fft.h"

FFT::FFT(){}

void FFT::compute_FFT(std::vector<std::complex<double>>& data){
    _size_of_signal = data.size();

    if (_size_of_signal <= 1) return;

    std::vector<std::complex<double>> even(_size_of_signal/2);
    std::vector<std::complex<double>> odd(_size_of_signal/2);

    for(int i = 0; i < _size_of_signal / 2; i++){
        even[i] = data[i * 2];
        odd[i] = data[i * 2 + 1];

    }

    compute_FFT(even);
    compute_FFT(odd);
//    _size_of_signal = data.size();

    for(int k = 0; k < (_size_of_signal / 2); ++k){


        std::complex<double> coef = std::polar(1.0, -2.0 * M_PI * k /_size_of_signal) * odd[k];

        data[k] = even[k] + coef;
        data[k + _size_of_signal / 2] = even[k] - coef;

    }

//    _data = data;

}

void FFT::perform_FFT(){
    compute_FFT(_data);
    sort_FFT();
}

void FFT::sort_FFT(){
    for(int h = 0; h < _data.size() / 2; h++){

    double abs_val =  std::abs(_data[h]);
     _abs.push_back(abs_val);

   //sample_freq*vec_1tosize/2*1/size
   _freq_val = (44100.0 * (h + 1)) * (1.0 / _data.size());
   //std::cout<< _freq_val <<std::endl;

   _freq_vec.push_back(_freq_val);
   //std::cout << "Freq_vec: " << _freq_vec[h] << std::endl;

   }

//    std::cout << "Size of abs_vec: " << _abs.size() << std::endl;

//    double max = *std::max_element(_abs.begin(), _abs.end());
//    std::cout << "Highest val in _abs:  " << max << std::endl;
//    std::cout << "Size of freq_: " << _freq_vec.size() << std::endl;
//    for(int f = 0; f < 10; f++){
//    std::cout << "content of abs_vec: " << _abs[f] << std::endl;
//    }
//    for(int f = 0; f < 10; f++){
//    std::cout << "content of _data_vec: " << _data[f] << std::endl;
//    }
//    auto it = std::find(_freq_vec.begin(), _freq_vec.end(), 0.502232);
//    if( it != _freq_vec.end()){
//    std::cout << "Abs val on freq 1477: " << "found at: " << it - _freq_vec.begin() + 1 << std::endl;
//    }
//    else {
//         std::cout << "Abs val on freq 1477: " << "not found " << it - _freq_vec.begin() + 1 <<  std::endl;
//    }
//    for(int f = 0; f < 10; f++){
//    std::cout << "content of freq_vec: " << _freq_vec[f] << std::endl;
//    }
//    std::cout << "Size of _data_vec: " << _data.size() << std::endl;
//    std::cout << "Start sort: " << std::endl;
   sort(_abs, _freq_vec);
//   for(int f = 0; f < 10 ; f++){
//   std::cout << "Highest 10 freq: " << _freq_vec[f] << std::endl;
//   }
}


void FFT::read_from_file(const std::string &fileName){
    std::ifstream inFile(fileName);
    std::string line;

    while (std::getline(inFile, line)) {
        double value = std::stod(line);
        _data.push_back(std::complex(value, 0.0));
       //std::cout << "Line: " << line << std::endl;
    }

    _actual_size_of_signal = _data.size();
     std::cout << "Size of signal: " << _actual_size_of_signal << std::endl;
    inFile.close();

    int y = std::log2(_actual_size_of_signal);
    _size_of_signal_with_zero = std::exp2(y + 1);
    for(int i = 0; i < (_size_of_signal_with_zero - _actual_size_of_signal); i++){
        _data.push_back(std::complex<double> (0.0, 0.0));
    }

}

std::vector<std::complex<double>> FFT::get_data(){
    return _data;
}

void FFT::sort(std::vector <double> &x, std::vector <double> &y){
    // Using the bubble sort algorithm
    // Evaluates x and swaps both x and y
    // The smallest value -> largest value

  std::vector<std::pair<double, double>> combined;
  for (size_t i = 0; i < x.size(); ++i) {
      combined.emplace_back(x[i], y[i]);
  }
   // Sort the combined vector by the first element in descending order
  std::sort(combined.begin(), combined.end(), [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
      return a.first > b.first;
  });

  // Unpack the sorted pairs back into the original vectors
  for (size_t i = 0; i < x.size(); ++i) {
      x[i] = combined[i].first;
      y[i] = combined[i].second;
  }

     std::cout << "Frequency found: " << y[0] << " and " << y[1] << std::endl;
}
