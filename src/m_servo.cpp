#include "m_servo.h"
HardwareSerial Serial1(PA10, PA9);
m_servo::m_servo(byte p = 0) : choose(1)
{

  Serial1.begin(115200);
  flag_serial = p;
}

void m_servo::senddata(char kk)
{
  char i;
  if (kk == 10)
  {
    for (i = 0; i < kk; i++)
    {
      Serial1.write(servo_sdata[i]);
    }
  }
  else if (kk == 4)
  {
    for (i = 0; i < kk; i++)
    {
      Serial1.write(servo_sdata1[i]);
    }
  }
}
/*閸楁洑閲滈懜鍨簚閹貉冨煑閸戣姤鏆�
閸欏倹鏆熼崣濠呅掗柌濠忕窗
    id_num: 閼稿灚婧€缂傛牕褰�0~255閿涘牓娅庨崢锟�0x79閵嗭拷0x7E閵嗭拷0x7F閿涘鍙�253娑撶嫪D閿涳拷;121閸欒渹璐熼獮鎸庢尡缂傛牕褰块敍灞藉祮閹碘偓閺堝鍩栭張娲厴娴兼艾鎼锋惔鏂垮煂閹貉冨煑閹稿洣鎶ら獮鑸靛⒔鐞涳拷
    angle: 閼稿灚婧€鐟欐帒瀹�0~270鎼达讣绱�         
    step:濮濄儲鏆熼敍宀冨煐閺堥缚顩︽潏鎯у煂閹稿洤鐣剧憴鎺戝閸掑棙顒為惃鍕偧閺佸府绱濇禒锟�10ms娑撳搫鎳嗛張鐔哥槨閸涖劍婀￠張鈧径姘崇箥閸旓拷3鎼达讣绱濆銉︽殶鐠佸墽鐤嗘稉琛♀偓锟�1閳ユ繃妞傞崚娆庝簰閺堚偓韫囶偊鈧喎瀹虫潪顒€濮╅妴锟�
    
濞夘煉绱扮拋鍓х枂鐎瑰本鐦崥搴礉鐠囬鏆€閸戦缚鍐绘径鐔荤箥鐞涘瞼娈戦弮鍫曟？
*/
void m_servo::set_angle(int id_num, float angle, int step)
{
  if (step <= 0)
    step = 1;

  if (angle < 0)
    angle = 0;
  if (angle > 285)
    angle = 285;
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[2] = (int)((int)(angle / 10)) % 100;
  servo_sdata[3] = ((int)(angle * 10)) % 100;
  servo_sdata[4] = (int)(step / 100);
  servo_sdata[5] = (int)((int)step % 100);
  servo_sdata[7] = 16;
  servo_sdata[8] = 1;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);

  servo_sdata[1] = id_num;
  servo_sdata[7] = 17;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
}

/* 鐠佸墽鐤嗛懜鍨簚濡€崇础閸戣姤鏆�

閼稿灚婧€閺堝绨茬粔宥喣佸锟�:鏉╂劕濮╁Ο鈥崇础閿涘矁鐤嗙€涙劖膩瀵骏绱濋梼璇插嚬濡€崇础閿涘矂鏀ｅ缁樐佸蹇ョ礉閹哄鏁稿Ο鈥崇础    閿涘牐绻嶉崝銊ュ挤鏉烆喖鐡欏Ο鈥崇础闁俺绻冩稉鎾崇潣閹稿洣鎶ょ€圭偟骞� 閸忔湹缍戞稉澶夐嚋闁俺绻冮張顒佹蒋閹稿洣鎶ょ€圭偟骞囬敍锟�
    婢跺嫪绨梼璇插嚬濡€崇础閺冭绱濋懜鍨簚閸欘垯浜掔悮顐ｅ箛閸旑煉绱濇担鍡樻Ц闂冭濮忓鍫濄亣閿涘矁鈧奔绗栨潪顒€濮╃搾濠傛彥閿涘矂妯嗛崝娑滅Ш婢讹拷
    婢跺嫪绨柨浣诡劥濡€崇础閺冭绱濋懜鍨簚閹貉冨煑缁嬪绨崥顖氬З閿涘苯鐨㈤懜鍨簚鐟欐帒瀹抽崶鍝勭暰閸︺劍鐓囨稉顏囶潡鎼达讣绱濇稉宥堝厴鐞氼偅骞囬崝锟�
    婢跺嫪绨幒澶屾暩濡€崇础閺冭绱濋懜鍨簚閸欘垯浜掔悮顐︽閹板繑骞囬崝顭掔礉闂冭濮忓鍫濈毈
    婢跺嫪绨潪顔肩摍濡€崇础閺冭绱濋懜鍨簚閸欘垯浜掗弫鏉戞噯鏉╂劘娴�
閸欏倹鏆熼崣濠呅掗柌锟�:
    id_num: 閼稿灚婧€缂傛牕褰�,閸楀疇顩︾拋鍓х枂缁楊剙鍤戦崣鐤煐閺堣櫣娈戝Ο鈥崇础閿涘矁绻栭柌灞藉讲娴犮儳鏁ら獮鎸庢尡瑜般垹绱￠幒褍鍩�
    mode_num: 閻€劍娼甸柅澶嬪娑撳秴鎮撻惃鍕佸锟�
        mode_num=1,闂冭鍑瑰Ο鈥崇础
        mode_num=2,闁夸焦顒村Ο鈥崇础
        mode_num=3,閹哄鏁稿Ο鈥崇础
        mode_num=4,鏉烆喖鐡欏Ο鈥崇础
*/
void m_servo::set_angles(int id_list[20], float angle_list[20], int step, int n)
{
  if (n == 1)
  {
    for (i = 0; i < 20; i++)
    {
      if (i != 0)
      {
        if (id_list[i] != 0)
          preset_angle(id_list[i], angle_list[i], step, 1);
      }
      else //i==0 闂傚倸鍊峰ù鍥р枖閺囥垹闂柨鏇炲€哥粻顖炴煥閻曞倹瀚�
      {
        preset_angle(id_list[i], angle_list[i], step, 1);
      }
      delay(2);
    }
    //濠电姷鏁告慨浼村垂婵傜ǹ鏄ラ柡宥冨妿閳瑰秵绻涘顔荤盎闁哄嫨鍎甸弻鈥愁吋鎼粹€崇闁诲孩鑹鹃ˇ鐢稿蓟閵娾晜鍋嗛柛灞剧☉椤忥拷 闂傚倸鍊风粈渚€骞夐敓鐘冲殞濡わ絽鍟€氬銇勯幒鍡椾壕濡炪値浜滈崯鎾极閹剧粯鏅搁柨鐕傛嫹 闂傚倷绀侀幖顐λ囬柆宥呯？闁圭増婢樼粈鍫熺箾閸℃ê濮堥柛娆忕箻閺屻劑鎮ら崒娑橆伓
    servo_sdata[1] = 121;
    servo_sdata[7] = 17;
    servo_sdata[8] = 1;
    servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
    senddata(10);
  }
  else if (n == 2)
  {
    for (i = 0; i < 20; i++)
    {
      if (i != 0)
      {
        if (id_list[i] != 0)
        {
          servo_sdata1[0] = 0x7E;
          servo_sdata1[1] = id_list[i];
          servo_sdata1[2] = (int)((int)angle_list[i] / 10) % 100;
          servo_sdata1[3] = (int)(angle_list[i] * 10) % 100;
          senddata(4);
          delay(2);
        }
      }
      else
      {
        servo_sdata1[0] = 0x7E;
        servo_sdata1[1] = id_list[i];
        servo_sdata1[2] = (int)((int)angle_list[i] / 10) % 100;
        servo_sdata1[3] = (int)(angle_list[i] * 10) % 100;
        senddata(4);
        delay(2);
      }
    }
    servo_sdata1[0] = 0x7E;
    servo_sdata1[1] = (int)(step / 100);
    servo_sdata1[2] = (int)((int)step % 100);
    servo_sdata1[3] = 0x7F;
    senddata(4);
    delay(2);
  }
}
void m_servo::change_mode(int id_num, int mode_num)
{
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[7] = 0x16;
  servo_sdata[8] = 0x10 + mode_num;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(10);
}



void m_servo::set_id(int id_num, int id_new)
{
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[2] = 0;
  servo_sdata[7] = 0x42;
  servo_sdata[8] = 0;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(2);
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[2] = id_new;
  servo_sdata[7] = 0x44;
  servo_sdata[8] = 0;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(10);
}



void m_servo::set_pid(int id_num, int pid)
{
  write_e2(id_num, 3, pid);
}

/*閼惧嘲褰囪ぐ鎾冲閼稿灚婧€閻樿埖鈧椒淇婇幁顖ょ礉娓氬顩цぐ鎾冲鐟欐帒瀹崇粵锟�
閼惧嘲褰囪ぐ鎾冲閼稿灚婧€閻樿埖鈧緤绱濋崠鍛
  閿涳拷1閿涘鍩栭張铏圭椽閸欙拷
  閿涳拷2閿涘缍嬮崜宥呯杽闂勫懓顫楁惔锟�
  閿涳拷3閿涘缍嬮崜宥嗘埂閺堟稖顫楁惔锟�
  閿涳拷4閿涘绻戦崶鐐跺煐閺堢儤澧嶉棁鈧潻鎰攽閺冨爼鏆遍敍鍧閿涳拷
  閿涳拷5閿涘鍩栭張铏规暩閸橈拷
  閿涳拷6閿涘鍩栭張铏规暩濞达拷
  閿涳拷7閿涘灏濋悧鍦芳

閸欏倹鏆熼崣濠呅掗柌锟�:
   id_num: 閼稿灚婧€缂傛牕褰�,閺屻儴顕楃粭顒€鍤戦崣鐤煐閺堣櫣娈戝Ο鈥崇础閿涘矁绻栭柌灞肩瑝閸欘垯浜掗悽銊ョ畭閹绢厽膩瀵骏绱欓搹鐣屽姧閹崵鍤庢稉濠傚涧閺堝绔存稉顏囧煐閺堢儤妞傞敍灞藉讲娴犮儰濞囬悽顭掔礉娴ｅ棜绻栭柌灞艰礋娴滃棔绗夐崙娲晩閿涘奔绗夌拋鈺冩暏閿涳拷
   para_num: 閹疇顩﹂弻銉嚄閻ㄥ嫬寮弫鎵椽閸欏嚖绱�
      para_num=0, 鏉╂柨娲栭幍鈧張澶変繆閹垳绮嶉幋鎰畱閸掓銆�
      para_num=1, 鏉╂柨娲栬ぐ鎾冲閼稿灚婧€缂傛牕褰�
      para_num=2, 鏉╂柨娲栬ぐ鎾冲鐟欐帒瀹�
      para_num=3, 鏉╂柨娲栬ぐ鎾冲閺堢喐婀滅憴鎺戝
      para_num=4, 鏉╂柨娲栭懜鍨簚閹碘偓闂団偓鏉╂劘顢戦弮鍫曟毐閿涘潰s閿涳拷
      para_num=5, 鏉╂柨娲栭懜鍨簚閻㈤潧甯�
      para_num=6, 鏉╂柨娲栭懜鍨簚閻㈠灚绁�
      para_num=7, 鏉╂柨娲栭懜鍨簚闁矮淇婂▔銏㈠閻滐拷
      para_num=8, 鏉╂柨娲栭懜鍨簚瑜版挸澧犲〒鈺佸
   o_m:(one or more) 閻€劍娼甸幐鍥ㄦ婢舵矮閲滈懜鍨簚(o_m=0)鏉╂ɑ妲告稉鈧稉顏囧煐閺堢尨绱濇俊鍌涚亯閸欘亝婀佹稉鈧稉顏囧煐閺堝搫褰叉禒銉╁櫚閻€劌绠嶉幘顓熌佸蹇ョ礉濮濄倖妞俹_m=1

Returns:
   鏉╂柨娲栭崐闂寸窗閺嶈宓乸ara_num閻ㄥ嫬鈧偐娴夋惔鏃€鏁奸崣锟�
      para_num=0, 鏉╂柨娲栭幍鈧張澶変繆閹垳绮嶉幋鎰畱閸掓銆�
      para_num=1, 鏉╂柨娲栬ぐ鎾冲閼稿灚婧€缂傛牕褰�
      para_num=2, 鏉╂柨娲栬ぐ鎾冲鐟欐帒瀹�
      para_num=3, 鏉╂柨娲栬ぐ鎾冲閺堢喐婀滅憴鎺戝
      para_num=4, 鏉╂柨娲栭懜鍨簚閹碘偓闂団偓鏉╂劘顢戦弮鍫曟毐閿涘潰s閿涳拷
      para_num=5, 鏉╂柨娲栭懜鍨簚閻㈤潧甯�
      para_num=6, 鏉╂柨娲栭懜鍨簚閻㈠灚绁�
      para_num=7, 鏉╂柨娲栭懜鍨簚闁矮淇婂▔銏㈠閻滐拷
      para_num=8, 鏉╂柨娲栭懜鍨簚瑜版挸澧犲〒鈺佸
Raises:
   婵″倹鐏夋径姘嚋閼稿灚婧€鏉╃偞甯撮弮璁圭礉閺屻儴顕楅幐鍥︽姢娴ｈ法鏁ゆ禍鍡楃畭閹绢厽膩瀵骏绱濇导姝砮turn 0,娑撳秵澧界悰灞剧叀鐠囥垺瀵氭禒锟�
*/

float m_servo::get_state(int id_num, int para_num, int o_m)
{
  //double servo_rpara[9]={0};
  u_numm = 0;
  if (id_num == 121 && o_m == 0)
  {
    return 0;
  }
  else if (id_num == 121 && o_m > 1)
  {
    return 0;
  }
  else
  {
    u_numm = 0;

    //濠电姷鏁搁崑鐘诲箵椤忓棛绀婇柍褜鍓熼弻娑欐償閳藉棛鍚嬮悗瑙勬礃椤ㄥ懓鐏掗梺鐓庮潟閸婃劙寮搁弽褜娓婚柕鍫濇鐏忕敻鏌涢悩鍐插闁诡喚鍋炵粋鎺斺偓锝庡亞閸欏棗鈹戦悙鏉戠仸妞ゎ厼娲幃姗€鏌嗗鍡欏幐婵炶揪缍佸褔鍩€椤掍胶绠撴い鏇秮椤㈡岸鍩€椤掑嫬绠犳繝濠傜墛椤ュ牊绻涢幋鐐茬瑨鐎垫澘绉归弻锝嗘償閵忊晛鏅遍梺鍝ュУ鐢€崇暦閹达附鏅搁柨鐕傛嫹
    servo_sdata[0] = 123;
    servo_sdata[1] = id_num;
    servo_sdata[7] = 0x13;

    servo_sdata[9] = 125;
    servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
    senddata(10);

    //delay(20);  //闂傚倸鍊风粈渚€骞栭锕€鐤い鎰堕檮閸嬪鏌ｉ幘鍐差唫婵炴垯鍨圭粻浼村箹鏉堝墽宀涢柡瀣墪閳规垿鎮欓崣澶嗘灆婵炲瓨绮嶉悧鐘茬暦閹达箑绠婚柟棰佺劍閸嶉潧顪冮妶鍡楀Ё缂佽鲸娲滅划璇差潩閼哥鎷洪梺鍛婄箓鐎氬嘲危婵傚憡鐓曢幖娣€曢崥褰掓煙楠炲灝鐏叉鐐差儔閺佸啴鍩€椤掑嫭鍊峰┑鐘叉处閻撶喖鏌熼柇锕€澧伴柟鐣屽У缁绘盯宕煎┑鍫濈厽闂佸搫鐬奸崰鎾诲箯閻樿鐏抽柧蹇ｅ亞娴滆埖绻濋悽闈涗沪闁瑰憡鎮傞幃銉︾附缁嬭法鐣鹃悗鍏夊亾闁告洖鐏氶弲锝夋⒑缂佹ɑ鐓ュ鐟帮工鍗辨い鏍仦閳锋帡鏌涚仦鎹愬濞存粈鍗抽弻鐔煎礄閵堝棗顏� 闂傚倸鍊峰ù鍥х暦閻㈢ǹ纾绘繛鎴烆焸濞戞ǚ鍫柛顐亝閺呪晠姊虹涵鍛涧缂佺姵鍨块幆灞轿旀担鍏哥盎闂佸搫绉查崝搴ㄥ煀閺囥垺鐓涢柛鈾€鏅涘顕€鏌＄仦鍓ф创妤犵偞甯￠幃娆撳级閹搭厺绱�15ms
    data = "";
    char i = 0;
    int t = 1000;
    bool n_s = 1;
    char num = 16;
    while (Serial1.available() < num && t > 0)
    {
      t -= 1;
      if (Serial1.available() > 0 && n_s)
      {
        if (Serial1.read() == 123)
        {
          n_s = 0;
          //data+='{';
          Para.servo_rdata[0] = 123;
          u_numm = 1;
        }
      }
    }
    while (Serial1.available() > 0)
    {
      //char inchar=(char)Serial1.read();
      //data+=inchar;
      Para.servo_rdata[u_numm] = (int)Serial1.read();
      u_numm++;
    }
    if (u_numm == num) //if(data.length()==num)
    {
      //data.toCharArray(servo_rdata,16);
      id_number = Para.servo_rdata[1];
      Para.cur_angle = (double)(Para.servo_rdata[2] * 10 + Para.servo_rdata[3] * 0.1);
      Para.exp_angle = (double)(Para.servo_rdata[4] * 10 + Para.servo_rdata[5] * 0.1);
      rem_step = Para.servo_rdata[6] * 10 + Para.servo_rdata[7];
      Para.servo_rpara[0] = id_number;
      Para.servo_rpara[1] = Para.cur_angle;
      Para.servo_rpara[2] = Para.exp_angle;
      Para.servo_rpara[3] = rem_step;
      Para.servo_mode = Para.servo_rdata[8];
      Para.cur_Voltage = Para.servo_rdata[9] + Para.servo_rdata[10] / 100;        //闂傚倷绀侀幖顐λ囬锕€鐤炬繝濠傜墕閽冪喖鏌曟繛鍨壄婵炲樊浜濋弲鎼佹煥閻曞倹瀚� 闂傚倸鍊峰ù鍥磻閹邦厾浠氶梻浣筋嚙缁绘垿鎳濇ィ鍐ㄧ厴闁告劦鍠栭悞鍨亜閹烘垵鈧綊宕伴幇鐗堢厽婵°倐鍋撻柣妤€锕︽禍鎼侇敇閻戝棛鍞甸悷婊冪箲缁绘盯鍩€椤掑嫭鐓欐い鏃囧吹閻瑦淇婇銏犳殭闁宠棄顦灒缁炬澘褰夌槐锟�
      Para.cur_current = Para.servo_rdata[11] / 10 + Para.servo_rdata[12] / 1000; //闂傚倷绀侀幖顐λ囬锕€鐤炬繝濠傜墕閽冪喖鏌曟繛鍨壄婵炲樊浜濋弲鎼佹煥閻曞倹瀚� 闂傚倸鍊峰ù鍥磻閹邦厾浠氶梻浣筋嚙缁绘垿鎳濇ィ鍐ㄧ厴闁告劦鍠栭悞鍨亜閹烘垵鈧綊宕伴幇鐗堢厽婵°倐鍋撻柣妤€锕︽禍鎼侇敇閻戝棛鍞甸悷婊冪箲缁绘盯鍩€椤掑嫭鐓欐い鏃囧吹閻瑧鈧娲栫紞濠囥€侀弴銏犖ч柛鎰╁妺閸狅拷

      if (Para.servo_rdata[13] == 0x01) //闂傚倷绀侀幖顐λ囬锕€鐤炬繝濠傜墕閽冪喖鏌曟繛鍨壄婵炲樊浜濋弲鎼佹煥閻曞倹瀚� 婵犵數濮烽弫鎼佸磻濞戔懞鍥敇閵忕姴鐎紓鍌欑劍椤洭鎮甸崼鏇熺厵闁绘垶锕╁▓鏇㈡煕鐎ｎ偆绠為柡灞诲姂閹倝宕掑☉姗嗕紦
        Para.baud_rate = 19200;
      else if (Para.servo_rdata[13] == 0x02)
        Para.baud_rate = 57600;
      else if (Para.servo_rdata[13] == 0x03)
        Para.baud_rate = 115200;
      else if (Para.servo_rdata[12] == 0x04)
        Para.baud_rate = 230400;
      else if (Para.servo_rdata[13] == 0x05)
        Para.baud_rate = 500000;
      else if (Para.servo_rdata[13] == 0x06)
        Para.baud_rate = 1000000;

      Para.cur_temprature = Para.servo_rdata[14] / 4 + Para.servo_rdata[14] % 4 * 0.25 + 10; //闂傚倷绀侀幖顐λ囬锕€鐤炬繝濠傜墕閽冪喖鏌曟繛鍨壄婵炲樊浜濋弲鎼佹煥閻曞倹瀚� 闂傚倸鍊峰ù鍥磻閹邦厾浠氶梻浣筋嚙缁绘垿鎳濇ィ鍐ㄧ厴闁告劦鍠栭悞鍨亜閹烘垵鈧綊宕伴幇鐗堢厽婵°倐鍋撻柣妤€锕︽禍鎼侇敇閻戝棛鍞甸悷婊冪箲閹便劑骞橀鍢夈儵鏌熼悜姗嗘當闁绘挻绋戦湁闁挎繂鎳庨ˉ蹇涙煕閻戝洦瀚�

      Para.servo_rpara[4] = Para.cur_Voltage;
      Para.servo_rpara[5] = Para.cur_current;
      Para.servo_rpara[6] = Para.baud_rate;
      Para.servo_rpara[7] = Para.cur_temprature;
      Para.servo_rpara[8] = Para.servo_mode;
      if (para_num == 1)
        return id_number;
      else if (para_num == 2)
        return Para.cur_angle;
      else if (para_num == 3)
        return Para.exp_angle;
      else if (para_num == 4)
        return rem_step;
      else if (para_num == 0)
      {
      }
      else if (para_num == 5)
        return Para.cur_Voltage;
      else if (para_num == 6)
        return Para.cur_current;
      else if (para_num == 7)
        return Para.baud_rate;
    }
    else
      return -1; //闂傚倷绀侀幖顐λ囬锕€鐤炬繝濠傜墕閽冪喖鏌曟繛鍨壄婵炲樊浜滈崘鈧銈嗘尵閸嬬偤鎮楅鍕拺缂備焦锕╁▓鏃堟煟濡も偓濡稓鍒掓繝鍥ㄦ櫇闁逞屽墴閳ワ箓宕稿Δ鈧粻姘舵煙濞堝灝鏋ょ紓鍫ヤ憾閺岋綁鎮㈤崫銉﹀殏闂佺懓鍤栭幏锟�  闂傚倸鍊风粈渚€骞夐敓鐘冲仭妞ゆ牜鍋涢崹鍌炴煟濡も偓閻楀繘銆呴柨瀣闁瑰鍋熼。鏌ユ倵濮橆厼鍝洪柟顔肩秺瀹曞爼濡搁妷褏銈风紓鍌欑劍瑜板啴鎮樺┑瀣р偓锕傚锤濡も偓缁犳岸鏌熷▓鍨灓缂傚牓浜堕弻锝夋偄閸濄儲鍤傜紓浣哄У閹告悂顢氶敐鍡欘浄閻庯綆浜為崐鐐烘偡濠婂啰绠崇紒顔垮吹閹瑰嫭绺介挊澶岀Ш闁轰焦鍔樼粻娑㈠棘濞嗗彞绱�
  }
}
/*鏉烆喖鐡欏Ο鈥崇础娑撳顔曠純顔垮煐閺堣櫣娈戞潪顒€濮╅柅鐔峰

閸欏倹鏆熼崣濠呅掗柌濠忕窗
    id_num: 閼稿灚婧€缂傛牕褰�,閸楀疇顩︾拋鍓х枂缁楊剙鍤戦崣鐤煐閺堣櫣娈戝Ο鈥崇础閿涘矁绻栭柌灞藉讲娴犮儳鏁ら獮鎸庢尡濡€崇础
    speed: 閺堥缚娴嗛崝銊┾偓鐔峰閿涳拷-1000~1000閿涘绱濋柅鐔峰娑撶儤顒滈懜鍨簚濮濓綀娴嗛敍灞藉冀娑斿寮芥潪锟�

*/
void m_servo::set_speed(int id_num, int speed)
{
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  if (speed > 0)
    servo_sdata[2] = 0x21;
  else if (speed < 0)
  {
    servo_sdata[2] = 0x22;
    speed = -speed;
  }
  else
    servo_sdata[2] = 0x20;
  servo_sdata[4] = (int)(speed / 10);
  servo_sdata[5] = (int)((speed) % 10);
  servo_sdata[7] = 0x20;
  servo_sdata[8] = 1;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;

  senddata(10);
  delay(10);
}
/*鏉烆喖鐡欏Ο鈥崇础娑撳顔曠純顔垮煐閺堣櫣娈戞潪顒€濮╅幍顓犵叐

閸欏倹鏆熼崣濠呅掗柌濠忕窗
    id_num: 閼稿灚婧€缂傛牕褰�,閸楀疇顩︾拋鍓х枂缁楊剙鍤戦崣鐤煐閺堣櫣娈戝Ο鈥崇础閿涘矁绻栭柌灞藉讲娴犮儳鏁ら獮鎸庢尡濡€崇础
    current:閼煎啫娲块敍锟�-5~5閿涘绱濋幍顓犵叐娑撶儤顒滈敍宀€鏁搁張鐑橆劀鏉烆剨绱濋崣宥勭閸欏秷娴嗛妴锟�
*/
void m_servo::set_torque(int id_num, float torque)
{
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  if (torque > 0)
    servo_sdata[2] = 0x21;
  else if (torque < 0)
  {
    servo_sdata[2] = 0x22;
    torque = -torque;
  }

  servo_sdata[4] = (int)(torque * 10);
  servo_sdata[5] = ((int)(torque * 100) % 10);
  servo_sdata[7] = 0x21;
  servo_sdata[8] = 1;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(10);
}
/* 娣囶喗鏁糉LASH娑擃厾娈戦懜鍨簚閸欏倹鏆熼崐锟�

閸欏倹鏆熼崣濠呅掗柌濠忕窗
    id_num: 闂団偓鐟曚椒鎱ㄩ弨绗稬ASH閼稿灚婧€缂傛牕褰�,婵″倹鐏夋稉宥囩叀闁挸缍嬮崜宥堝煐閺堣櫣绱崣鍑ょ礉閸欘垯浜掗悽锟�121楠炴寧鎸遍敍灞肩稻閺勵垵绻栭弮鑸碘偓鑽ゅ殠娑撳﹤褰ч懗鍊熺箾娑撯偓娑擃亣鍩栭張鐚寸礉閸氾箑鍨径姘嚋閼稿灚婧€娴兼俺顫︾拋鍓х枂閹存劗娴夐崥宀€绱崣锟�
    address: flash閸︽澘娼�
                    Address  Item          describe              閸掓繂顫愰崐锟�
                    00                 ID 閼稿灚婧€ID閸欙拷               0x00   00
                    01                 BAUDRATE 濞夈垻澹掗悳鍥偓澶嬪       0x03   03
                    02                 ZERO_OFFSET  鐟欐帒瀹抽崑蹇曅�   0xB9 185
                    03                 Angle_Kp 鐟欐帒瀹砅ID  P閸欏倹鏆�   0x14 20
                    04                 Angle_Ki 鐟欐帒瀹砅ID  I閸欏倹鏆�   0x05 0
                    05                 Angle_Kd 鐟欐帒瀹砅ID  D閸欏倹鏆�   0x28 40
                    06                 MaxDuty  閺堚偓婢堆冨窗缁岀儤鐦梽鎰煑  0x64 100
                    07                 MinDuty  閺堚偓鐏忓繐宕扮粚鐑樼槷闂勬劕鍩�  0x0A 10
                    08                 PosError 閻╊喗鐖ｆ担宥呯摍鐎瑰綊鏁�   0x03 3
                    09                 InitMode 娑撳﹦鏁稿Ο鈥崇础       0x10 16
                    10                 StallLimit 閸絻娴嗘穱婵囧Б鐟欐帒瀹�   0x50 80
                    11                 Current_Kp    閸旀稓鐓╅幒褍鍩桺ID P閸欏倹鏆�    0x05  5
                    12                 Free1             妫板嫮鏆€1          0x00  0
                    13                 Free2             妫板嫮鏆€2          0x00  0
    value: flash娑擃厼顕惔鏂挎勾閸р偓閻ㄥ嫯顔曠純顔炬窗閺嶅洤鈧拷

*/
void m_servo::write_e2(int id_num, int address, int value)
{
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[7] = 0x42;
  servo_sdata[8] = 0;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(10);
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[2] = (int)(value % 100);
  servo_sdata[3] = (int)(value / 100);
  servo_sdata[7] = 0x44;
  servo_sdata[8] = address;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(10);
}

/* 閻犲洩顕цぐ鍣乴ash濞戞搩鍘界€垫氨鈧鐭紞鍛磾椤旂偓鐣遍柤绋跨仛濠р偓闁告瑥鍊归弳鐔煎磹閿燂拷

闁告瑥鍊归弳鐔煎矗婵犲憛鎺楁煂婵犲繒绐�
    id_num: 闁肩ǹ鐏氬┃鈧紓鍌涚墪瑜帮拷,闁告鐤囬々锔炬媼閸撗呮瀭缂佹鍓欓崵鎴﹀矗閻ゎ垰鐓愰柡鍫ｆ濞堟垵螣閳ュ磭纭€闁挎稑鐭佺换鏍煂鐏炶棄璁插ù鐘劤閺併倝鐛幐搴㈠啊婵☆垪鈧磭纭€
    address: flash濞戞搩鍘惧▓鎴︽儎缁嬫鍤犲ù锝呯Ф閻ゅ棝鏁嶉敓锟�0濞寸媴缍€閵嗭拷0x00, 濞寸姰鍎查婵堢尵缂佹ê鑵归柨娑樻湰閻︹剝绋夐埀顒佹媴瀹ュ洦鐣遍柛蹇氭腹缂嶅宕ラ锛勭枀濠碘€冲€风粭鍛偘閿燂拷
                    Address Item       describe                         闁告帗绻傞～鎰板磹閿燂拷
                    00                 ID 闁肩ǹ鐏氬┃鈧琁D闁告瑱鎷�                      0x00   00
                    01                 BAUDRATE 婵炲鍨绘竟鎺楁偝閸ヮ兘鍋撴径瀣仴              0x03   03
                    02                 ZERO_OFFSET  閻熸瑦甯掔€规娊宕戣箛鏇咃拷            0xB9   185
                    03                 Angle_Kp 閻熸瑦甯掔€圭爡ID  P闁告瑥鍊归弳锟�          0x14   20
                    04                 Angle_Ki 閻熸瑦甯掔€圭爡ID  I闁告瑥鍊归弳锟�          0x00   0
                    05                 Angle_Kd 閻熸瑦甯掔€圭爡ID  D闁告瑥鍊归弳锟�          0x28   40
                    06                 MaxDuty  闁哄牃鍋撳鍫嗗啫绐楃紒宀€鍎ら惁顕€姊介幇顒€鐓�          0x64   100
                    07                 MinDuty  闁哄牃鍋撻悘蹇撶箰瀹曟壆绮氶悜妯兼Х闂傚嫭鍔曢崺锟�          0x0A   10
                    08                 PosError 闁烩晩鍠楅悥锝嗘媴瀹ュ懐鎽嶉悗鐟扮秺閺侊拷            0x03   3
                    09                 InitMode 濞戞挸锕﹂弫绋课熼垾宕囩                0x10   16
                    10                 StallLimit 闁割偂绲诲ù鍡樼┍濠靛洤袘閻熸瑦甯掔€癸拷          0x50   80
                    11                 Current_Kp 闁告梹绋撻悡鈺呭箳瑜嶉崺妗篒D P闁告瑥鍊归弳锟�     0x05   5
                    12                 Free1      濡澘瀚弳鈧�1                 0x00   0
                    13                 Free2      濡澘瀚弳鈧�2                 0x00   0
                    14                 Product_Number  濞存籂鍐╂儌閻犲洤妫楅崺鍡涘矗閿燂拷       0x06   6
                    15                 Hardware_Version   缁绢収鍏涘▎銏ゆ偋閸喐鎷�      0x01   1
                    16                 Software_Version   閺夌儐鍨▎銏ゆ偋閸喐鎷�      0x03   3
Returns:
    閺夆晜鏌ㄥú鏍嚋閸偅绨氶柛娆愬灴閳ь兛娴囩换鍐级閵壯勭暠16闁轰胶澧楀畵渚€濡撮敓锟�

*/
int m_servo::read_e2(int id_num, int address)
{
  u_numm = 0;

  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[7] = 0x45;
  servo_sdata[8] = address;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(2);
  data = "";
  char i = 0;
  int t = 1000;
  bool n_s = 1;
  char num = 16;
  while (Serial1.available() < num && t > 0)
  {
    t -= 1;
    if (Serial1.available() > 0 && n_s)
    {
      if (Serial1.read() == 123)
      {
        n_s = 0;
        //data+='{';
        Para.servo_rdata[0] = 123;
        u_numm = 1;
      }
    }
  }
  while (Serial1.available() > 0)
  {
    //char inchar=(char)Serial1.read();
    //data+=inchar;
    Para.servo_rdata[u_numm] = (int)Serial1.read();
    u_numm++;
  }
  if (u_numm == num)
  {
    // char Para.servo_rdata[16];
    //data.toCharArray(Para.servo_rdata,16);

    id_number = Para.servo_rdata[1];
    return (Para.servo_rdata[2] * 100 + Para.servo_rdata[3]);
  }
}
/* 閻犲洩顕цぐ鍣乴ash濞戞搩鍘界€垫氨鈧鐭紞鍛磾椤旂偓鐣遍柤绋跨仛濠р偓闁告瑥鍊归弳鐔煎磹閿燂拷

闁告瑥鍊归弳鐔煎矗婵犲憛鎺楁煂婵犲繒绐�
    id_num: 闁肩ǹ鐏氬┃鈧紓鍌涚墪瑜帮拷,闁告鐤囬々锔炬媼閸撗呮瀭缂佹鍓欓崵鎴﹀矗閻ゎ垰鐓愰柡鍫ｆ濞堟垵螣閳ュ磭纭€闁挎稑鐭佺换鏍煂鐏炶棄璁插ù鐘劤閺併倝鐛幐搴㈠啊婵☆垪鈧磭纭€
    address: flash濞戞搩鍘惧▓鎴︽儎缁嬫鍤犲ù锝呯Ф閻ゅ棝鏁嶉敓锟�0濞寸媴缍€閵嗭拷0x00, 濞寸姰鍎查婵堢尵缂佹ê鑵归柨娑樻湰閻︹剝绋夐埀顒佹媴瀹ュ洦鐣遍柛蹇氭腹缂嶅宕ラ锛勭枀濠碘€冲€风粭鍛偘閿燂拷
                    Address Item       describe                         闁告帗绻傞～鎰板磹閿燂拷
                    00                 ID 闁肩ǹ鐏氬┃鈧琁D闁告瑱鎷�                      0x00   00
                    01                 BAUDRATE 婵炲鍨绘竟鎺楁偝閸ヮ兘鍋撴径瀣仴              0x03   03
                    02                 ZERO_OFFSET  閻熸瑦甯掔€规娊宕戣箛鏇咃拷            0xB9   185
                    03                 Angle_Kp 閻熸瑦甯掔€圭爡ID  P闁告瑥鍊归弳锟�          0x14   20
                    04                 Angle_Ki 閻熸瑦甯掔€圭爡ID  I闁告瑥鍊归弳锟�          0x00   0
                    05                 Angle_Kd 閻熸瑦甯掔€圭爡ID  D闁告瑥鍊归弳锟�          0x28   40
                    06                 MaxDuty  闁哄牃鍋撳鍫嗗啫绐楃紒宀€鍎ら惁顕€姊介幇顒€鐓�          0x64   100
                    07                 MinDuty  闁哄牃鍋撻悘蹇撶箰瀹曟壆绮氶悜妯兼Х闂傚嫭鍔曢崺锟�          0x0A   10
                    08                 PosError 闁烩晩鍠楅悥锝嗘媴瀹ュ懐鎽嶉悗鐟扮秺閺侊拷            0x03   3
                    09                 InitMode 濞戞挸锕﹂弫绋课熼垾宕囩                0x10   16
                    10                 StallLimit 闁割偂绲诲ù鍡樼┍濠靛洤袘閻熸瑦甯掔€癸拷          0x50   80
                    11                 Current_Kp 闁告梹绋撻悡鈺呭箳瑜嶉崺妗篒D P闁告瑥鍊归弳锟�     0x05   5
                    12                 Free1      濡澘瀚弳鈧�1                 0x00   0
                    13                 Free2      濡澘瀚弳鈧�2                 0x00   0
                    14                 Product_Number  濞存籂鍐╂儌閻犲洤妫楅崺鍡涘矗閿燂拷       0x06   6
                    15                 Hardware_Version   缁绢収鍏涘▎銏ゆ偋閸喐鎷�      0x01   1
                    16                 Software_Version   閺夌儐鍨▎銏ゆ偋閸喐鎷�      0x03   3
Returns:
    閺夆晜鏌ㄥú鏍嚋閸偅绨氶柛娆愬灴閳ь兛娴囩换鍐级閵壯勭暠16闁轰胶澧楀畵渚€濡撮敓锟�

*/
void m_servo::read_e2_all(int id_num)
{
  u_numm = 0;

  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[7] = 0x46;
  servo_sdata[8] = choose;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(2);
  data = "";
  char i = 0;
  int t = 1000;
  bool n_s = 1;
  char num = 16;
  while (Serial1.available() < num && t > 0)
  {
    t -= 1;
    if (Serial1.available() > 0 && n_s)
    {
      if (Serial1.read() == 123)
      {
        n_s = 0;
        //data+='{';
        Para.servo_rdata[0] = 123;
        u_numm = 1;
      }
    }
  }
  while (Serial1.available() > 0)
  {
    //char inchar=(char)Serial1.read();
    //data+=inchar;
    Para.servo_rdata[u_numm] = (int)Serial1.read();
    u_numm++;
  }
  if (u_numm == num)
  {
    //char Para.servo_rdata[16];
    //data.toCharArray(Para.servo_rdata,16);
  }
}

/*
閻熸瑱缍侀弨锝夋嚋閸偅绨氬ǎ鍥ㄧ箖婵垽鎮╅懜纰樺亾娓氬﹦绀夐柛锔哄姀閸╂牠寮垫ウ璺ㄧ闁稿繈鍎扮换姘跺箮閵堝洤笑闁诡兛璁查埀顒佹煛閳ь剚妫佺换鍐╂姜閺傘倗绀勯弶鈺佹穿濞村洦绌卞┑鍥戦柕鍡曡兌閺佹悂宕㈢€ｂ晝绠介柟韬插€戦埀顑跨劍娣囶垱鎯旈敂璺ㄧ闁硅翰鍊х槐姘跺触鎼搭垳绀夊☉鎾崇Т閹奸攱鎯旈弮鍥х厫闁哄牓缂氬ù鍡涘礉閵婏箑鐦瑰ù鐘€х槐锟�
闁肩ǹ鐏氬┃鈧柛銉у仩椤曚即宕ｆ繝浣瑰弿闁衡偓缁楃ìASH闁圭ǹ娲ｉ幎銈夊矗椤栨瑥鈻忛柣鈧妸閳ь剙鍊稿﹢顏堝箳閹烘鐝熼悗浣冨閸ぱ囨嚋閸偅绨氬ǎ鍥ㄧ箖婵垽鎯冮崟顐㈡枾闁搞儳濮撮幃妤呮晬閸絿顦伴弶鐐差潟閳ь兛妞掔欢鐢告偨閻㈠灚鏆╅柛妯侯儍閳ь兛鐒︽穱顖涙償閿曚胶绠栧Δ鍌涳公缁辨岸鏁嶇仦钘夎闂侇偅淇虹换鍐嫉椤掍焦钂嬮柟绋挎矗閹躲倝骞嬮弽顓炴闁哄倿顣︾粭鍌炴偨娴ｅ唭鎺楁煥娴ｉ绠介柟韬插€楁慨鎼佸箑娴ｇ鍋撻敓锟�

閺夆晛娲╁ù鍥ㄧ┍濠靛洤袘闁挎稑鑻畵鍡涙嚋閸偅绨氶柛銉уЬ缁€瀣姜閸婄喓绠栧鍫嗗棛绠婚柛蹇嬪劥閸ゆ粓骞嬮幋婊呯闁硅翰鍊楁慨鎼佸箑娴ｇ鍋撻敓锟�
闁汇垽娼х敮鍥ㄧ┍濠靛洤袘闁挎稑鐭侀崺鏍嫉閸濆嫬甯掗悹浣告憸濞堟垶绗熷☉姘毄闁肩厧鍟ú鎸庣▔閿燂拷6.0-12v闁挎稑濂旂粭澶愬捶閵婎煈鍤夐柤鐓庡暙濞插潡宕橀崨顓炵仧閺夆晜绋戦崣鍡涙偨闂堟稑绔惧ǎ鍥ㄧ箖婵垽鎮╅懜纰樺亾娴ｇ鍋撻敓锟�
婵炴挴鏅涚€硅櫕绌卞┑鍥戦柨娑樿嫰濠€顏堟⒐閹稿孩顦ч梻鍌涙綑閵囧洨鎷归悢缁樼グ閺夆晜鍔樺ù鍡涘箣閺嵮呭閺夌儐鍓欓閬嶆嚊绾惧鐓愰柡鍫ョ細缁诲啴鎮滈銊х閺夆晜绋戦崣鍡椼€掗埡浣割唺濞ｅ洦绻冩慨銏ゆ偐閼哥鍋撴笟濠勭濮掓稒枪椤撹崵鎲撮敃鈧ぐ鍌氥€掗埡浣割唺濞ｅ洦绻冩慨銏ゅ磹闂傜绀�60闁斥晛鍟ㄩ埀顒€鍋婄槐娆戞喆閿曗偓瑜板倹绌卞┑鍥戦柛濠勫帶瑜版煡鏌呭宕囩畺濞ｅ浂鍠楅弫绯塋ASH閻犱礁澧介悿鍡涙晬婢跺牃鍋撻敓锟�

闁告瑥鍊归弳鐔煎矗婵犲憛鎺楁煂婵犲繒绐�
    id_num: 闁肩ǹ鐏氬┃鈧紓鍌涚墪瑜帮拷,閻熸洑娴囪闂佸じ鑳跺▓鎴﹀及椤栨粠鍎戦柛鎴犲Т瑜板潡鎳滈崹顐ｇ皻闁靛棴鎷�
Returns:
    闁哄喛鎷�
*/
void m_servo::unlock_stall(int id_num)
{
  servo_sdata[0] = 123; //0x7B
  servo_sdata[1] = id_num;
  servo_sdata[2] = 13;
  servo_sdata[3] = 50;
  servo_sdata[4] = 0;
  servo_sdata[5] = 1;
  servo_sdata[7] = 0x30;
  servo_sdata[8] = 0;
  servo_sdata[9] = 125; //0x7D
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  //闂傚倸鍊风粈渚€骞夐敓鐘冲仭闁挎洖鍊搁崹鍌炴煕瑜庨〃鍛存倿閸偁浜滈柟杈剧稻绾爼鏌涢弬璺ㄐч柡宀嬬節瀹曞崬鈻庤箛鏃備簴缂傚倷娴囨ご鍝ユ暜閿熺姴鏋侀柟鍓х帛閺呮悂鏌ㄩ悤鍌涘
  senddata(10);
  delay(20);
}
/*
闁告帗绻傞～鎰板礌閺堝粯ASH

闁告瑥鍊归弳鐔煎矗婵犲憛鎺楁煂婵犲繒绐�
    id_num: 闁肩ǹ鐏氬┃鈧紓鍌涚墪瑜帮拷,闁告鐤囬々锕傚礆濠靛棭娼楅柛鏍ㄦ箟LASH濞戞搩鍙冨▍搴㈢閸滅瘚闁告瑩顥撳▓鎴﹀箥閳ь剟寮垫径濞惧亾绾绀凢LASH闁革附婢樺鍐矗婵犲倸鏁堕悗鍦攰椤棝宕¤箛姘煎敶闁挎稑顧€缁辨繃绋夊鍛濞寸姰鍎辩粻宥夊箻閿燂拷
*/
void m_servo::e2_init(int id_num)
{
  for (i = 0; i < 10; i++)
  {
    servo_sdata[i] = 0;
  }
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[7] = 0x42;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(20);
  servo_sdata[0] = 123;
  servo_sdata[1] = id_num;
  servo_sdata[7] = 0x43;
  servo_sdata[9] = 125;
  servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
  senddata(10);
  delay(20);
}
/*
閻犱礁澧介悿鍡涙嚋閸偅绨氭慨婵撶到閻栬泛顔忛妷銈囩▕闁汇劌瀚〒鑸殿殗濡鍒婇幖杈炬嫹

闁告瑥鍊归弳鐔煎矗婵犲憛鎺楁煂婵犲繒绐�
    id_num: 闁肩ǹ鐏氬┃鈧紓鍌涚墪瑜帮拷,闁告鐤囬々锔炬媼閸撗呮瀭闁告垹濮磋ぐ鍧楁嚋閸偅绨氶柣銊ュ娴兼劖鎷呭⿰鍕垔閹艰揪鎷�
    temprature:闁哄牃鍋撳Δ鍌浬戞穱顖涙償閿旇偐绀夐柛妤€鐤囬崺鏍嫉閸濆嫪绱ｅù锝嗙矎缁诲啰绮欑€ｂ晞鍘繛鎾櫅鐎硅櫕顨囧Ο鍦壘閻犲洢鍎叉穱顖涙償閿旇姤顦ч柨娑樼焷缁绘﹢宕楅妷锔垮垔閹艰揪缂氱换姘跺箮閵堝繒绀夋慨婵勫€栧鍌炴嚋閸偅绨氶柛娆樹海閸忔﹢宕堕悙棰濆殺闁轰胶澧楀畵渚€鏁嶇仦鑲╃憹閺夆晜鍔樺ù锟�
*/
void m_servo::temprature_set(int id_num, int temprature)
{

  write_e2(id_num, 12, temprature);
}

void m_servo::preset_angle(int id_num, float angle, int step, int rn)
{
  if (step <= 0)
    step = 1;
  if (rn == 0)
  {
    if (angle < 0)
      angle = 0;
    if (angle > 285)
      angle = 285;
    servo_sdata[0] = 123;
    servo_sdata[1] = id_num;
    servo_sdata[2] = (int)((int)(angle / 10)) % 100;
    servo_sdata[3] = ((int)(angle * 10)) % 100;
    servo_sdata[4] = (int)(step / 100);
    servo_sdata[5] = (int)((int)step % 100);
    servo_sdata[7] = 16;
    servo_sdata[8] = 1;
    servo_sdata[9] = 125;
    servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
    senddata(10);

    servo_sdata[1] = id_num;
    servo_sdata[7] = 17;
    servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
    senddata(10);
  }
  else if (rn == 1)
  {
    if (angle < 0)
      angle = 0;
    if (angle > 285)
      angle = 285;
    servo_sdata[0] = 123;
    servo_sdata[1] = id_num;
    servo_sdata[2] = (int)((int)(angle / 10)) % 100;
    servo_sdata[3] = ((int)(angle * 10)) % 100;
    servo_sdata[4] = (int)(step / 100);
    servo_sdata[5] = (int)((int)step % 100);
    servo_sdata[7] = 16;
    servo_sdata[8] = 1;
    servo_sdata[9] = 125;
    servo_sdata[6] = (servo_sdata[1] + servo_sdata[2] + servo_sdata[3] + servo_sdata[4] + servo_sdata[5] + servo_sdata[7] + servo_sdata[8]) % 100;
    senddata(10);
  }
}
