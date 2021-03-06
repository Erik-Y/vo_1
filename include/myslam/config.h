/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam 
{
class Config
{
private:
    static std::shared_ptr<Config> config_ ; //静态数据成员(类外初始化)config_是指向Config类的共享智能指针
    cv::FileStorage file_;                  //file_是cv::FileStorage类型的对象，用于读取yaml文件
    Config () {} // private constructor makes a singleton
    
public:
    
    ~Config();  // close the file when deconstructing 
    
    // 静态成员函数，可以通过作用域运算符调用，创建config_并将config_->file_与文件路径、名绑定
    static void setParameterFile( const std::string& filename ); 
    
    // 静态成员函数，可以通过作用域运算符调用
    template< typename T >
    static T get( const std::string& key )
    {
        return T( Config::config_->file_[key] );
    }
};



}

#endif // CONFIG_H
