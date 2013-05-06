#include <dae.h>

#ifndef NO_ZAE
#include <fstream>
#include <dae/daeErrorHandler.h>
#include <dae/daeZAEUncompressHandler.h>

//-----------------------------------------------------------------
const std::string daeZAEUncompressHandler::MANIFEST_FILE_NAME("manifest.xml");
const std::string daeZAEUncompressHandler::MANIFEST_FILE_ROOT_ELEMENT_NAME("dae_root");
const int daeZAEUncompressHandler::CASE_INSENSITIVE = 2;
const int daeZAEUncompressHandler::BUFFER_SIZE = 1024;
const std::string daeZAEUncompressHandler::EMPTY_STRING = "";

//-----------------------------------------------------------------
daeZAEUncompressHandler::daeZAEUncompressHandler( const daeURI& zaeFile )
  : mZipFile(NULL)
  , mZipFileURI(zaeFile)
  , mValidZipFile(false)
  , mRootFilePath("")
{
    std::string zipFilePath = cdom::uriToNativePath(zaeFile.getURI());
    mZipFile = unzOpen(zipFilePath.c_str());

    mValidZipFile = mZipFile != NULL;

    mTmpDir = cdom::getSafeTmpDir() + cdom::getRandomFileName() + 
        cdom::getFileSeparator() + mZipFileURI.pathFile() + cdom::getFileSeparator();
}

//-----------------------------------------------------------------
daeZAEUncompressHandler::~daeZAEUncompressHandler()
{
    if (mZipFile != NULL)
        unzClose(mZipFile);
}

//-----------------------------------------------------------------
const std::string& daeZAEUncompressHandler::obtainRootFilePath()
{
    if (!isZipFile())
        return EMPTY_STRING;

    if (boost::filesystem::create_directories(mTmpDir))
    {
        if (extractArchive(mZipFile, mTmpDir))
        {
            if (retrieveRootURIFromManifest(mTmpDir))
            {
                return mRootFilePath;
            }
            else
            {
                // TODO find root file without manifest
            }
        }
        else
        {
            daeErrorHandler::get()->handleError("Error extracting archive in daeZAEUncompressHandler::obtainRootFilePath\n");
        }
    }
    else
    {
        daeErrorHandler::get()->handleError("Error creating tmp dir in daeZAEUncompressHandler::obtainRootFilePath\n");
    }

    boost::filesystem::remove_all(this->getTmpDir());
    return EMPTY_STRING;
}

//-----------------------------------------------------------------
bool daeZAEUncompressHandler::retrieveRootURIFromManifest(const std::string& tmpDir)
{
    // extract via libxml.
    bool error = false;
    xmlTextReaderPtr xmlReader = xmlReaderForFile(
        (tmpDir + MANIFEST_FILE_NAME).c_str(),
        NULL,
        0
        );

    if (xmlReader)
    {
        if (findManifestRootElement(xmlReader))
        {
            if (xmlTextReaderRead(xmlReader))
            {
                if (xmlTextReaderNodeType(xmlReader) == XML_READER_TYPE_TEXT) {
                    const xmlChar* xmlText = xmlTextReaderConstValue(xmlReader);

                    // copy xmlText.
                    std::string rootFilePath((daeString)xmlText);

                    // destroy xmlText.
                    xmlTextReaderRead(xmlReader);

                    cdom::trimWhitespaces(rootFilePath);
                    mRootFilePath = cdom::nativePathToUri(tmpDir + rootFilePath);
                }
                else
                {
                    error = true;
                }
            }
            else
            {
                error = true;
            }
        }
        else
        {
            error = true;
        }
    }
    else
    {
        error = true;
    }

    if (xmlReader)
        xmlFreeTextReader(xmlReader);
    if (error)
    {
        daeErrorHandler::get()->handleError("Error parsing manifest.xml in daeZAEUncompressHandler::retrieveRootURIFromManifest\n");
        return false;
    }

    return true;
}

//-----------------------------------------------------------------
bool daeZAEUncompressHandler::findManifestRootElement( xmlTextReaderPtr xmlReader )
{
    while(xmlTextReaderNodeType(xmlReader) != XML_READER_TYPE_ELEMENT)
    {
        if (xmlTextReaderRead(xmlReader) != 1) {
            return false;
        }
    }

    daeString elementName = (daeString)xmlTextReaderConstName(xmlReader);
    if (strcmp(elementName, MANIFEST_FILE_ROOT_ELEMENT_NAME.c_str()) == 0)
    {
        return true;
    }
    return findManifestRootElement(xmlReader);
}

//-----------------------------------------------------------------
bool daeZAEUncompressHandler::extractArchive( unzFile zipFile, const std::string& destDir )
{
    bool error = false;
    unz_global_info globalZipInfo;

    if (unzGetGlobalInfo (zipFile, &globalZipInfo) == UNZ_OK)
    {
        for (unsigned int i=0; i<globalZipInfo.number_entry; ++i)
        {
            if (!extractFile(zipFile, destDir))
            {
                error = true;
                break;
            }

            if ((i+1)<globalZipInfo.number_entry)
            {
                if (unzGoToNextFile(zipFile) != UNZ_OK)
                {
                    daeErrorHandler::get()->handleError("Error moving to next file in zip archive in daeZAEUncompressHandler::extractArchive\n");
                    error = true;
                    break;
                }
            }
        }
    }
    else
    {
        daeErrorHandler::get()->handleError("Error getting info for zip archive in daeZAEUncompressHandler::extractArchive\n");
        error = true;
    }
    return !error;
}

//-----------------------------------------------------------------
bool daeZAEUncompressHandler::extractFile( unzFile zipFile,  const std::string& destDir )
{
    bool error = false;

    unz_file_info fileInfo;
    char currentFileName[256]; // ARGH !!!
    int fileInfoResult = unzGetCurrentFileInfo(zipFile, &fileInfo, currentFileName, sizeof(currentFileName), 0, 0, 0, 0);
    if (fileInfoResult == UNZ_OK)
    {
        if ( currentFileName[ strlen(currentFileName)-1 ] == '/')
        {
            if (!boost::filesystem::create_directories(destDir + currentFileName))
            {
                daeErrorHandler::get()->handleError("Error creating dir from zip archive in daeZAEUncompressHandler::extractFile\n");
                error = true;
            }
        }
        else
        {
            if (unzOpenCurrentFile(zipFile) == UNZ_OK)
            {

                char* buffer = 0;
                int readBytes = 1;
                buffer = new char[ BUFFER_SIZE ];
                std::string currentOutFilePath(destDir + std::string(currentFileName));
                std::ofstream outFile(currentOutFilePath.c_str(), std::ios::binary);

                while (readBytes > 0)
                {
                    readBytes = unzReadCurrentFile(zipFile, buffer, BUFFER_SIZE);
                    outFile.write(buffer, readBytes);
                }
                delete[] buffer;
                outFile.close();

                if (readBytes >= 0)
                {
                    if (unzCloseCurrentFile(zipFile) == UNZ_CRCERROR)
                    {
                        daeErrorHandler::get()->handleError("CRC error while opening file in zip archive in daeZAEUncompressHandler::extractFile\n");
                        error = true;
                    }
                    else
                    {
                        if (!checkAndExtractInternalArchive(currentOutFilePath))
                        {
                            error = true;
                        }
                    }
                }
                else
                {
                    daeErrorHandler::get()->handleError("Error reading file in zip archive in daeZAEUncompressHandler::extractFile\n");
                    error = true;
                }

            }
            else
            {
                daeErrorHandler::get()->handleError("Error opening file in zip archive in daeZAEUncompressHandler::extractFile\n");
                error = true;
            }
        }
    }
    else
    {
        daeErrorHandler::get()->handleError("Error getting info for file in zip archive in daeZAEUncompressHandler::extractFile\n");
        error = true;
    }

    return !error;
}

//-----------------------------------------------------------------
bool daeZAEUncompressHandler::checkAndExtractInternalArchive( const std::string& filePath )
{
    unzFile zipFile = unzOpen(filePath.c_str());
    if (zipFile == NULL)
    {
        // TODO check for other compression formats.
        return true;
    }

    bool error = false;

    boost::filesystem::path archivePath(filePath);
    std::string dir = archivePath.branch_path().string();

    const std::string& randomSegment = cdom::getRandomFileName();
    std::string tmpDir = dir + cdom::getFileSeparator() + randomSegment + cdom::getFileSeparator();
    if (boost::filesystem::create_directory(tmpDir))
    {
        if (!extractArchive(zipFile, tmpDir))
        {
            daeErrorHandler::get()->handleError("Could not extract internal zip archive in daeZAEUncompressHandler::checkAndExtractInternalArchive\n");
            error = true;
        }
    }
    else
    {
        daeErrorHandler::get()->handleError("Could not create temporary directory for extracting internal zip archive in daeZAEUncompressHandler::checkAndExtractInternalArchive\n");
        error = true;
    }

    unzClose(zipFile);

    if (!error)
    {
        if (boost::filesystem::remove(archivePath))
        {
            boost::filesystem::rename(tmpDir, archivePath);
        }
        else
        {
            daeErrorHandler::get()->handleError("Could not remove internal zip archive in daeZAEUncompressHandler::checkAndExtractInternalArchive\n");
            error = true;
        }
    }

    return !error;
}

#endif //NO_ZAE
