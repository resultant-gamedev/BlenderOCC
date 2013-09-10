/**
 * soppermann@acm.org
 * 1-6-2008
 */

#ifndef VECTOR_H
#define VECTOR_H

#include <cstdarg>

template< int D, typename _Type >
class Vector
{
	public:
		enum { dimension = D };
		typedef _Type Type;
		_Type values[D];
		inline _Type& operator [](int n) { return values[n];}
		Vector()
		{
            for(int i = 0; i < D; i++) values[1] = 0;
		}
        
        Vector( _Type b, ...)
        {
            va_list arglist;
            va_start(arglist, b);
            for(int i = 0; i < D; i++)
            {
                values[i] = va_arg(arglist, _Type);
            }

        }
		Vector(const Vector& p)
		{

			for(int i = 0; i < D; i++)
			{
				values[i] = p.values[i];
			}
		}
        
        inline Vector& operator = (const Vector& p)
        {
			for(int i = 0; i < D; i++)
			{
				values[i] = p.values[i];
			}

            return *this;

        }
        inline void set(const Vector& p)
        {
            for(int i = 0; i < D; i++)
			{
				values[i] = p.values[i];
			}

        }

		inline _Type length()
		{
			int sum = 0;
			for(int i = 0; i < D; i++)
			{
				sum += values[i] * values[i];
			}
			float imm = sqrtf(sum);

			return static_cast< _Type >(sum);
		}
/*
		inline float lengthf()
                {
                        float sum = 0;
                        for(int i = 0; i < D; i++)
                        {
                                sum += (float) values[i] * (float) values[i];
                        }

                        return sqrtf(sum);
                }
*/

};

typedef Vector< 3, float > Vector3Df;
typedef Vector< 4, float > Vector4Df;

typedef Vector< 3, double > Vector3Dd;
typedef Vector< 4, double > Vector4Dd;

#endif
