/* Este header deve ser incluso na main e utilizado na task do display */

//tamanhos em pixels
#define heart_img_width 20
#define heart_img_height 20
#define step_img_width 20
#define step_img_height 20
#define lowbat_img_width 18
#define lowbat_img_height 16
#define chargbat_img_width 18
#define chargbat_img_height 18

/* imagem low-battery */
const uint8_t lowbat_img_bitmap[] = {
  0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x30, 0x00, 0x00, 0x18, 0x00, 
  0xF8, 0xFF, 0x01, 0x0C, 0x0C, 0x01, 0x0E, 0x8C, 0x01, 0x0A, 0x86, 0x01, 
  0x0A, 0xC6, 0x01, 0x0E, 0xC3, 0x01, 0x0C, 0xE3, 0x01, 0xF8, 0xFF, 0x01, 
  0x80, 0x01, 0x00, 0xC0, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

/* imagem bateria carregando */
const uint8_t chargbat_img_bitmap[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xFC, 0x7F, 0x00, 0x02, 0x80, 0x00, 0x02, 0x80, 0x00, 0x02, 0x80, 0x01, 
  0xC2, 0x01, 0x01, 0x02, 0x07, 0x01, 0x02, 0x80, 0x01, 0x02, 0x80, 0x00, 
  0x06, 0xC0, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* imagem passos */
const uint8_t step_img_bitmap[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xE0, 0x01, 0x00, 
  0xF0, 0x01, 0x00, 0xE0, 0x01, 0x00, 0xF0, 0x19, 0x00, 0xF0, 0x78, 0x00, 
  0xE0, 0xF0, 0x00, 0xE0, 0xF8, 0x00, 0xE0, 0xF8, 0x00, 0xE0, 0xF1, 0x00, 
  0xE0, 0xF1, 0x00, 0xC0, 0x61, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 
  0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 
  };

/* imagem coracao */
const uint8_t heart_img_bitmap[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF9, 0x00, 
  0xF8, 0xFF, 0x01, 0xFC, 0xFF, 0x03, 0xFC, 0xFF, 0x03, 0xFC, 0xFF, 0x03, 
  0xFC, 0xFF, 0x03, 0xFC, 0xFF, 0x03, 0xF8, 0xFF, 0x01, 0xF8, 0xFF, 0x01, 
  0xF0, 0xFF, 0x00, 0xE0, 0x7F, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x0F, 0x00, 
  0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };