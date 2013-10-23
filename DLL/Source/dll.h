#ifndef _DLL_H_
#define _DLL_H_

/* Begin User-Defined */
#define export extern "C" __declspec (dllexport)
/* End User Defined */

#if BUILDING_DLL
# define DLLIMPORT __declspec (dllexport)
#else /* Not BUILDING_DLL */
# define DLLIMPORT __declspec (dllimport)
#endif /* Not BUILDING_DLL */


class DLLIMPORT DllClass
{
  public:
    DllClass();
    virtual ~DllClass(void);

  private:

};


#endif /* _DLL_H_ */
