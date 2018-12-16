#ifndef ARITHMETIC_CODEC
#define ARITHMETIC_CODEC

#include <stdio.h>
#include "config.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class Adaptive_Bit_Model                     // adaptive model for binary data
{
public:

	Adaptive_Bit_Model(void);

	void reset(void);                             // reset to equiprobable model
	
public:  //  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
	void     update(void);
	double   bit_0_prob;
	unsigned update_cycle, bits_until_update, bit_0_count, bit_count;
	friend class Arithmetic_Codec;
};

struct Node
{
	unsigned bit;
	unsigned state;
	double weight;
	double current_weight;
	double length;
	double value;
	double base;
	unsigned char* ac_pointer;
#if ISADAPTIVE
	Adaptive_Bit_Model m;
	unsigned decodedNum;
	unsigned errorNum;
	double errorPro;
#endif
	Node* parent;
};
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - Class definitions - - - - - - - - - - - - - - - - - - - - - - - - - - -

class Static_Bit_Model                         // static model for binary data
{
public:

  Static_Bit_Model(void);

  void set_probability_0(double);             // set probability of symbol '0'

private:  //  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  double bit_0_prob;
  friend class Arithmetic_Codec;
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class Static_Data_Model                       // static model for general data
{
public:

  Static_Data_Model(void);
 ~Static_Data_Model(void);

  unsigned model_symbols(void) { return data_symbols; }

  void set_distribution(unsigned number_of_symbols,
                        const double probability[] = 0);    // 0 means uniform

private:  //  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  unsigned data_symbols;
  double * distribution;                            // cumulative distribution
  friend class Arithmetic_Codec;
};



                    
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class Adaptive_Data_Model                   // adaptive model for general data
{
public:

  Adaptive_Data_Model(void);
  Adaptive_Data_Model(unsigned number_of_symbols);
 ~Adaptive_Data_Model(void);

  unsigned model_symbols(void) { return data_symbols; }

  void reset(void);                             // reset to equiprobable model
  void set_alphabet(unsigned number_of_symbols);

private:  //  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  void       update(void);
  double   * distribution;                          // cumulative distribution
  unsigned * symbol_count;                     // counter of symbol occurences
  unsigned   data_symbols, total_count, update_cycle, symbols_until_update;
  friend class Arithmetic_Codec;
};


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - Encoder and decoder class - - - - - - - - - - - - - - - - - - - - - - -

// Class with both the arithmetic encoder and decoder.  All compressed data is
// saved to a memory buffer.

class Arithmetic_Codec
{
public:


  Arithmetic_Codec(void);
 ~Arithmetic_Codec(void);
  Arithmetic_Codec(unsigned max_code_bytes,
                   unsigned char * user_buffer = 0);         // 0 = assign new

  unsigned char * buffer(void) { return code_buffer; }

  void set_buffer(unsigned max_code_bytes,
                  unsigned char * user_buffer = 0);          // 0 = assign new

  void     start_encoder(void);
  void     start_decoder(void);
  void     read_from_file(FILE * code_file);  // read code data, start decoder
  void     read_from_file(double* dParity, unsigned int encoderLen);

  unsigned stop_encoder(void);                 // returns number of bytes used
  unsigned write_to_file(FILE * code_file);   // stop encoder, write code data
  unsigned write_to_file(bool* dacParity, int p0Num);

  void     stop_decoder(void);

  void     put_bit(unsigned bit);
  unsigned get_bit(void);

  void     put_bits(unsigned data, unsigned number_of_bits);
  unsigned get_bits(unsigned number_of_bits);

  void     encode(unsigned bit,
                  Static_Bit_Model &);
  unsigned decode(Static_Bit_Model &);

  void     encode(unsigned data,
                  Static_Data_Model &);
  unsigned decode(Static_Data_Model &);

  void decode(Node** node,Static_Bit_Model & M, unsigned char* crc);

  bool checkCRC(int * source, const int length, unsigned char crc);
  void checkDecode(Node* node,unsigned char crc);
  void     encode(unsigned bit,
                  Adaptive_Bit_Model &);
  unsigned decode(Adaptive_Bit_Model &);
  void decode(Node** node, Adaptive_Bit_Model & M,double overlap_input, unsigned char* crc);

  void     encode(unsigned data,
	  Adaptive_Data_Model &);
  unsigned decode(Adaptive_Data_Model &);
  void setdecoder(unsigned nodecount,double error,double p0,char* nblock,unsigned termination);
  void updateError(Node &N, int state);
  void setblock(int block_size,int max_node);
  void nextnblock(unsigned count);
  void setoverlap(double overlap,Static_Bit_Model & M);
  void setoverlap(double overlap, Adaptive_Bit_Model & M);

  void setweight(double px,double py);
  double getpy(char *nblock, int n);

  void setweight();

private:  //  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  void     propagate_carry(void);
  void     renorm_enc_interval(void);
  void     renorm_dec_interval(void);
  unsigned buffer_size, mode;     // mode: 0 = undef, 1 = encoder, 2 = decoder
  double   base, value, length;           // arithmetic coding state variables
  unsigned char * code_buffer, * new_buffer, * ac_pointer;

  //new parameter for decoder
  double weight_0to1;//=log(cross_probability)+log(1-M.bit_0_prob);
  double weight_0to0;//=log(1-cross_probability)+log(M.bit_0_prob);
  double weight_1to1;//=log(1-cross_probability)+log(1-M.bit_0_prob);
  double weight_1to0;//=log(cross_probability)+log(M.bit_0_prob);
  
  double error;
  double p0;

  double weight_ab_0to1;//=log(cross_probability)+log(1-M.bit_0_prob);
  double weight_ab_0to0;//=log(1-cross_probability)+log(M.bit_0_prob);
  double weight_ab_1to1;//=log(1-cross_probability)+log(1-M.bit_0_prob);
  double weight_ab_1to0;//=log(cross_probability)+log(M.bit_0_prob);

  double crc_wrong_weight;

  char* nblock;
  unsigned blocklength;
  unsigned blockcount;
  double overlap;
  unsigned TERMINATION;

  int block_size; //this for decode block£¬in order to fairly compare with our scheme.
  int max_node;
  double pow_0;
  double pow_1;
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#endif
