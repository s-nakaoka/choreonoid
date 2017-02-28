/**
   @author Kunio Kojima
*/
#include "PreviewControlPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;

// ownerBodyItem = findOwnerItem<BodyItem>(); // we may be able to get bodyItem from body

MatrixXd smoothMatrix( const int rows, int halfRange ){
  if( 2*halfRange + 1 > rows )halfRange = 1;

  MatrixXd smoothMat = MatrixXd::Identity( rows, rows );
  for( int i = 0; i < halfRange; ++i ){
    for( int j = 1; j-1 < halfRange; ++j ){
      smoothMat(i,max(0,i-j)) = 1.0/(i+1+halfRange); smoothMat(i,i) = 1.0/(i+1+halfRange); smoothMat(i,min((int)rows-1,i+j)) = 1.0/(i+1+halfRange); 
    }
  }
  for( int i = halfRange; i < rows - halfRange; ++i ){
    for( int j = 1; j-1 < halfRange; ++j ){
      smoothMat(i,i-j) = 1.0/(2*halfRange+1); smoothMat(i,i) = 1.0/(2*halfRange+1); smoothMat(i,i+j) = 1.0/(2*halfRange+1); 
    }
  }
  for( int i = rows - halfRange; i < rows; ++i ){
    for( int j = 1; j-1 < halfRange; ++j ){
      smoothMat(i,max(0,i-j)) = 1.0/(rows-i+halfRange);  smoothMat(i,i) = 1.0/(rows-i+halfRange); smoothMat(i,min((int)rows-1,i+j)) = 1.0/(rows-i+halfRange); 
    }
  }

  return smoothMat;
}

// 球面からZ座標を計算
double PreviewControlPlugin::calcZFromSphere(const Vector3d centerPos, const Vector3d pos, const double radius ){
  return centerPos.z() + sqrt( pow(radius,2) - pow(pos.x()-centerPos.x(), 2) - pow(pos.y()-centerPos.y(), 2) );
}

// 腰位置を可動域内に修正
void PreviewControlPlugin::modifyWaistIntoRange
( Vector3d& waistPos, const Vector3d lFootPos, const Vector3d rFootPos, const Vector3d lHipPos, const Vector3d rHipPos, const double legLength ){
  // void modifyWaistIntoRange( Position waistPos, const Vector3d lFootPos, const Vector3d rFootPos, const Vector3d lHipPos, const Vector3d rHipPos, const double legLength ){
  // const double ratio = 1;
  const double ratio = 0.99;
  double z = waistPos.z();
  waistPos.z() = min( waistPos.z(),
                      min( waistPos.z() - lHipPos.z() + calcZFromSphere( lFootPos, lHipPos, legLength*ratio ),
                           waistPos.z() - rHipPos.z() + calcZFromSphere( rFootPos, rHipPos, legLength*ratio ) ) );
  // cout << " lhippos:" << lHipPos.z() << "  sphere z(l):" << calcZFromSphere( lFootPos, lHipPos, legLength*ratio ) << endl;
  // cout << " rhippos:" << rHipPos.z() << "  sphere z(r):" << calcZFromSphere( rFootPos, rHipPos, legLength*ratio ) << endl;
  if( waistPos.z() < z ) cout << "down waist " << z-waistPos.z() << endl;
}

  // 予見制御
void PreviewControlPlugin::PreviewControl(){
  cout << "\x1b[31m" << "Start Preview Control" << "\x1b[m" << endl;

  // BodyItem,BodyPtr
  ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->checkedItems<BodyItem>();
  BodyPtr body = bodyItems[0]->body();// ロボットモデルは１個のみチェック


  // 足先リンク名取得
  Link *lFootLink, *rFootLink;
  UtilPlugin::getFootLink( &lFootLink, &rFootLink, body );

  // PoseSeqItem,BodyMotionItem,BodyMotionPtr
  ItemList<PoseSeqItem> poseSeqItems = ItemTreeView::mainInstance()->selectedItems<Item>();
  BodyMotionItem* bodyMotionItem = poseSeqItems[0]->bodyMotionItem();// poseSeqを１個のみ選択
  BodyMotionPtr motion = bodyMotionItem->motion();
  // boost::filesystem::path poseSeqPath( AppConfig::archive()->get("currentFileDialogDirectory", shareDirectory()).c_str() );
  string poseSeqPathString = poseSeqItems[0]->filePath();
  boost::filesystem::path poseSeqPath(poseSeqPathString);
  cout << " parent_path:" << poseSeqPath.parent_path().string() << " basename:" << getBasename(poseSeqPath) << endl;

  JointPathPtr jpl = getCustomJointPath( body, body->rootLink(), lFootLink );
  JointPathPtr jpr = getCustomJointPath( body, body->rootLink(), rFootLink );


  // 脚長さ計算
  for( int i = 0; i < jpl->numJoints(); ++i ) jpl->joint(i)->q() =0;
  body->calcForwardKinematics();
  double legLength = ( jpl->joint(0)->p() - jpl->endLink()->p() ).norm();
  cout << "legLength: " << legLength << endl;

  // motion生成
  cout << "numFrames: " << motion->numFrames() << endl;
  if( motion->numFrames() == 0 ){
    BodyMotionGenerationBar* bmgb = BodyMotionGenerationBar::instance();// インスタンス生成
    PoseProvider* provider = poseSeqItems[0]->interpolator().get();
    bmgb->shapeBodyMotion(body, provider, bodyMotionItem, true);
    cout << "Generated motion" << endl;
  }else{
    cout << "Need not generate motion" << endl;
  }

  double dt = 1.0/motion->frameRate(), max_tm = motion->numFrames() / (double)motion->frameRate();

  // 目標zmpを計算
  PoseSeqInterpolatorPtr poseSeqInterpolatorPtr = poseSeqItems[0]->interpolator();
  Vector3SeqPtr refZmpSeqPtr;
  // refZmpSeqPtr.reset( new Vector3Seq() );
  // refZmpSeqPtr->setNumFrames(motion->numFrames(), true);
  // for (size_t i = 0; i < static_cast<size_t>(round(max_tm / dt)); i++) {
  //   poseSeqInterpolatorPtr->seek( i * dt );

  //   // poseSeqのZMP
  //   refZmpSeqPtr->at(i).x() = (*(poseSeqInterpolatorPtr->ZMP())).x();
  //   refZmpSeqPtr->at(i).y() = (*(poseSeqInterpolatorPtr->ZMP())).y();
  //   refZmpSeqPtr->at(i).z() = (*(poseSeqInterpolatorPtr->ZMP())).z();
  // }
  // refZmpSeqPtr = motion->ZMPseq();// motionのZMP
  // refZmpSeqPtr = motion->getOrCreateExtraSeq<Vector3Seq>("ZMP");// motionのZMP
  refZmpSeqPtr = motion->extraSeq<Vector3Seq>("ZMP");// motionのZMP Vector3SeqはZMPSeqでもいい??
  cout << "Finished generating ref zmp seq" << endl;

  Vector3SeqPtr initialZMPSeqPtr;
  initialZMPSeqPtr.reset( new Vector3Seq() );
  initialZMPSeqPtr->setNumFrames(motion->numFrames(), true);
  DynamicsPlugin::calcZMP( body, motion, initialZMPSeqPtr );// 入力動作のzmpの計算

  // 目標zmp・初期zmp・初期重心軌道書き出し
  stringstream ss0;
  ss0 << poseSeqPath.parent_path().string() << "/" << getBasename(poseSeqPath) << "_plot_" << motion->frameRate() << "fps" << ".dat";
  FILE* fp0 = fopen(ss0.str().c_str(), "w");
  Vector3d lastPosVec = body->rootLink()->p();
  Vector3d lastVelVec = VectorXd::Zero(3);
  fprintf(fp0,"time initZMPX initZMPY refZMPX refZMPY COMX COMY rootPosX rootPosY rootVelX rootVelY rootAccX rootAccY\n");
  for(int i = 0; i < motion->numFrames(); ++i){
    motion->frame(i) >> *body; body->calcCenterOfMass();
      
    Vector3d velVec = (body->rootLink()->p() - lastPosVec)/dt;
    Vector3d accVec = (velVec - lastVelVec)/dt;
    lastPosVec = body->rootLink()->p(); lastVelVec = velVec;

    fprintf(fp0, "%f  %f %f  %f %f  %f %f  %f %f  %f %f  %f %f\n",
            i*dt, initialZMPSeqPtr->at(i).x(),initialZMPSeqPtr->at(i).y(), refZmpSeqPtr->at(i).x(),refZmpSeqPtr->at(i).y(),// 1  2 3(初期ZMP)  4 5(目標zmp)
            body->centerOfMass().x(), body->centerOfMass().y(), 
            body->rootLink()->p().x(), body->rootLink()->p().y(),
            velVec.x(), velVec.y(), accVec.x(), accVec.y() );
  }
  fclose(fp0);

  // 予見制御収束ループ
  Vector3SeqPtr zmpSeqPtr;
  for(int loopNum = 0; loopNum < 5; ++loopNum ){

    zmpSeqPtr.reset( new Vector3Seq() );
    zmpSeqPtr->setNumFrames(motion->numFrames(), true);
    DynamicsPlugin::calcZMP( body, motion, zmpSeqPtr );// 実際のzmpの計算

    // 予見制御用の実際のzmpと目標zmp、誤差zmp、時刻tmを計算
    // cout << max_tm << " " << dt << " " << round(max_tm / dt) << endl;
    std::queue<hrp::Vector3> ref_zmp_list;
    std::deque<double> tm_list;
    for (size_t i = 0; i < static_cast<size_t>(round(max_tm / dt)); ++i){

      // tm_list
      double tmp_tm = i * dt;
      tm_list.push_back(tmp_tm);
      // ref_zmp_list
      hrp::Vector3 ref_v;// 目標zmp
      ref_v << refZmpSeqPtr->at(i).x(), refZmpSeqPtr->at(i).y(), refZmpSeqPtr->at(i).z();
        
      hrp::Vector3 v;// 実際のzmp
      v << zmpSeqPtr->at(i).x(), zmpSeqPtr->at(i).y(), zmpSeqPtr->at(i).z();
      // zmp差分を入力
      if( i+1 < static_cast<size_t>(round(max_tm / dt)) ) ref_zmp_list.push(ref_v - v);
      else ref_zmp_list.push( ref_zmp_list.back() );// 最後は1つ前の値と同じ

      // cout << tmp_tm << "     " << ref_v.transpose()  << "     " << v.transpose() << endl;
    }
    cout << "Finished generating ref zmp list" << endl;
    
    Vector3SeqPtr diffCMSeqPtr;
    diffCMSeqPtr.reset( new Vector3Seq() );
    diffCMSeqPtr->setNumFrames(motion->numFrames(), true);

    // preview_dynamics_filter<preview_control> df(dt, 0.8, ref_zmp_list.front());
    rats::preview_dynamics_filter<rats::extended_preview_control> df(dt, 0.8, ref_zmp_list.front());
    double cart_zmp[3], refzmp[3];
    bool r = true;
    size_t index = 0;
    while (r) {

      hrp::Vector3 p, x;// p: current_refzmp   x: refcog
      // r = df.update(p, x, ref_zmp_list.front(), ref_frame_list.front(), !ref_zmp_list.empty());
      r = df.update(p, x, ref_zmp_list.front(), !ref_zmp_list.empty());
      if (r) {
        df.get_cart_zmp(cart_zmp);
        df.get_current_refzmp(refzmp);

        x.z() = 0;
        diffCMSeqPtr->at(index) = x;

        ++index;
        /* zmpx ;; this zmp is "zmp as a table-cart model" cogy refzmpx zmpy cogy refzmpy */
        // fprintf(fp, "%f %f %f %f %f %f %f\n", tm_list[index], cart_zmp[0], x[0], refzmp[0], cart_zmp[1], x[1],refzmp[1] );
        // cout << tm_list[index] << " " << x.transpose() << endl;
      } else if ( !ref_zmp_list.empty() ) r = true;
      if (!ref_zmp_list.empty()){
        ref_zmp_list.pop();
      }
    }
    cout << "Finished calculating ref diff centroid" << endl;

    for(int i = 0; i < motion->numFrames(); ++i){
      Vector3d lFootPos,rFootPos, lHipPos, rHipPos;
      Matrix3 lFootR,rFootR;
      // cout << i*dt << " ";
      motion->frame(i) >> *body;
      body->calcForwardKinematics();// 状態更新
        
      lFootPos = lFootLink->p(); lFootR = lFootLink->R();// 足先位置取得
      rFootPos = rFootLink->p(); rFootR = rFootLink->R();

      // cout << "before lhip:" << jpl->joint(0)->p.transpose() << " rhip:" << jpr->joint(0)->p.transpose() << endl;
      // cout << "before lfoot:" << lFootPos.transpose() << " rfoot:" << rFootPos.transpose() << endl;
      body->rootLink()->p() += 0.7*diffCMSeqPtr->at(i);// 腰位置修正 ゲインを掛けるだけでは微妙
      body->calcForwardKinematics();// 状態更新
      lHipPos = jpl->joint(0)->p(); rHipPos = jpr->joint(0)->p();
      // cout << " after lhip:" << jpl->joint(0)->p.transpose() << " rhip:" << jpr->joint(0)->p.transpose() << endl;
      // cout << " after lfoot:" << lFootPos.transpose() << " rfoot:" << rFootPos.transpose() << endl;
      Vector3 tmpVec = body->rootLink()->p();
      modifyWaistIntoRange( tmpVec, lFootPos, rFootPos, lHipPos, rHipPos, legLength );
      body->rootLink()->p() = tmpVec;// 腰位置修正は要改良
      body->calcForwardKinematics();// 状態更新

      if( !jpl->calcInverseKinematics(lFootPos,lFootR) ) cout << "\x1b[31m" << i*dt << " lfoot IK fail" << "\x1b[m" << endl;
      if( !jpr->calcInverseKinematics(rFootPos,rFootR) ) cout << "\x1b[31m" << i*dt << " rfoot IK fail" << "\x1b[m" << endl;

      // cout << body->rootLink()->p.transpose() << "  " << lFootPos.transpose() << "  " << rFootPos.transpose() << endl << endl;

      motion->frame(i) << *body;
    }
    cout << "Finished modifying waist position" << endl;

    // zmpファイル書き出し
    stringstream ss;
    ss << poseSeqPath.parent_path().string() << "/" << getBasename(poseSeqPath) << "_plot_" << motion->frameRate() << "fps_" << loopNum << ".dat";
    FILE* fp = fopen(ss.str().c_str(),"w"); if( fp == NULL ){ cout << "\x1b[31m" << "dat file open error" << "\x1b[m" << endl; return; }
    DynamicsPlugin::calcZMP( body, motion, zmpSeqPtr );
    fprintf(fp,"time ZMPX ZMPY COMX COMY rootPosX rootPosY\n");
    for(int i = 0; i < motion->numFrames(); ++i){
      motion->frame(i) >> *body; body->calcCenterOfMass();
      fprintf(fp, "%f  %f %f  %f %f  %f %f\n", 
              i*dt,  zmpSeqPtr->at(i).x(),zmpSeqPtr->at(i).y(),
              body->centerOfMass().x(),body->centerOfMass().y(), 
              body->rootLink()->p().x(), body->rootLink()->p().y() );
    }
    fclose(fp);
    cout << "Finished recording ZMP trajectory" << endl;

  }// modifying loop

  // motionに出力動作ZMPを代入
  Vector3SeqPtr finalZMPSeqPtr = motion->extraSeq<Vector3Seq>("ZMP");// motionのZMP
  for(int i = 0; i < motion->numFrames(); ++i){
    finalZMPSeqPtr->at(i) = zmpSeqPtr->at(i);
  }

  cout << "Finished assigning ZMP to motion class" << endl;

  cout << "\x1b[31m" << "Finished Preview Control" << "\x1b[m" << endl << endl;
}



CNOID_IMPLEMENT_PLUGIN_ENTRY(PreviewControlPlugin)
