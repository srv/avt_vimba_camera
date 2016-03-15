/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        MutexGuard.cpp

  Description: Implementation of a mutex helper class for locking and unlocking.
               (For internal use only)

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#include <VimbaCPP/Source/MutexGuard.h>

#include <VimbaCPP/Include/VimbaSystem.h>

namespace AVT {
namespace VmbAPI {

MutexGuard::MutexGuard()
{
}

MutexGuard::MutexGuard( MutexPtr pMutex )
{
    if ( SP_ISNULL( pMutex ))
    {
        LOG_FREE_TEXT( "No mutex passed." );
    }
    else
    {
        Protect( pMutex );
    }
}

MutexGuard::MutexGuard( BasicLockablePtr pLockable )
{
    if ( SP_ISNULL( pLockable ))
    {
        LOG_FREE_TEXT( "No mutex passed." );
    }
    else
    {
        Protect( pLockable );
    }
}

MutexGuard::MutexGuard( const BasicLockable &rLockable )
{
    Protect( rLockable );
}

MutexGuard::~MutexGuard()
{
    Release();
}

void MutexGuard::Protect( MutexPtr pMutex )
{
    if( SP_ISNULL( pMutex ))
    {
        LOG_FREE_TEXT( "No mutex passed." );
    }

    else if( SP_ISEQUAL( pMutex, m_pMutex ))
    {
        return;
    }

    Release();

    SP_ACCESS( pMutex )->Lock();
    m_pMutex = pMutex;
}

void MutexGuard::Protect( BasicLockablePtr pLockable )
{
    if( SP_ISNULL( SP_ACCESS( pLockable )->GetMutex() ))
    {
        LOG_FREE_TEXT( "No mutex passed." );
    }
    else
    {
        Protect( SP_ACCESS( pLockable )->GetMutex() );
    }
}

void MutexGuard::Protect( const BasicLockable &rLockable )
{
    Protect( rLockable.GetMutex() );
}

bool MutexGuard::Release()
{
    if( SP_ISNULL( m_pMutex ))
    {
        return false;
    }

    SP_ACCESS( m_pMutex )->Unlock();
    SP_RESET( m_pMutex );

    return true;
}

}} //namespace AVT::VmbAPI
