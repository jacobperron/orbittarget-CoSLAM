/***************************************************************************
 * Project: CoSLAM Target Following                                        *
 * Author:  Jens Wawerla (jwawerla@sfu.ca)                                 *
 ***************************************************************************
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 **************************************************************************/
#include "RapiStage"

#ifdef RAPI_GUI
#include "RapiGui"
#endif

#include "../orbittarget.h"


#define STAGE

using namespace Rapi;
void split( const std::string& text, const std::string& separators, std::vector<std::string>& words) {
  int n = text.length();
  int start, stop;
  start = text.find_first_not_of(separators);
  while ((start >= 0) && (start < n)) {
    stop = text.find_first_of(separators, start);
    if ((stop < 0) || (stop > n)) stop = n;
    words.push_back(text.substr(start, stop - start));
    start = text.find_first_not_of(separators, stop+1);
  }
}

extern "C" int Init ( Stg::Model* mod, Stg::CtrlArgs* args)
{
  // tokenize the argument string into words
  std::vector<std::string> words;
  split( args->worldfile, std::string(" \t"), words );
  assert( words.size() > 2 );
#ifdef RAPI_GUI
  CGui* gui = CGui::getInstance ( 0, NULL );
#endif

  printf ( "-----------------------------------\n" );
  printf ( "Chatterbox Oribt Moving Target \n" );
  printf ( "  build %s %s \n", __DATE__, __TIME__ );
  printf ( "  compiled against RAPI version %s (%s) build %s\n", RAPI_VERSION(),
           RAPI_GIT_VERSION(), RAPI_BUILD() );
  printf ( "\n" );

  CStageRobot* robot;
  COrbitTarget* robotCtrl;

  Stg::World* world = mod->GetWorld();
  Stg::Model* target = world->GetModel("target");
  Stg::Model* nn = world->GetModel(words[1].c_str());

  // init general stuff
  ErrorInit ( 1, false );
  initRandomNumberGenerator();

  // create robot and its controller
  robot = new CStageRobot ( mod );
  robot->setName(words[2]);

  if (words.size() > 3)
    robotCtrl = new COrbitTarget ( robot, words[3]);
  else
    robotCtrl = new COrbitTarget ( robot );

  robotCtrl->setStgTarget( target );
  robotCtrl->setStgNN( nn );
  
#ifdef RAPI_GUI
  gui->registerRobot ( robot );
#endif
  return 0; // ok
}
